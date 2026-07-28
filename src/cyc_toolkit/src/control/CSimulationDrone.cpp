#include "CSimulationDrone.h"
#include <algorithm>
#include <spdlog/fmt/fmt.h>
#include <math/CPolynomialFitting.h>

DroneState::DroneState()
    : state(Eigen::VectorXf::Zero(STATE_NUM))
{
}

DroneState::DroneState(Eigen::VectorXf x)
    : state(std::move(x))
{
    if (state.size() != STATE_NUM)
    {
        throw std::runtime_error{ fmt::format("DroneState state vector has wrong size: {} vs {}", state.size(), static_cast<int>(STATE_NUM)) };
    }
}

bool DroneState::from_vector(const Eigen::VectorXf& x)
{
    if (x.size() != state.size())
        return false;

    for (Eigen::Index i = 0; i < x.size(); ++i)
    {
        state[i] = x[i];
    }

    return true;
}

const Eigen::VectorXf DroneState::get_partial_state(Eigen::Index begin) const
{
    return state.segment(begin, STATE_DIMENSIONS);
}

bool DroneState::set_partial_state(Eigen::Index begin, const Eigen::VectorXf& x)
{
    if (x.size() != STATE_DIMENSIONS)
        return false;

    for (Eigen::Index i = 0; i < x.size(); ++i)
    {
        state[begin + i] = x[i];
    }

    return true;
}

void DroneSimulation::step(const Eigen::VectorXf& u, float dt)
{
    if (u.size() != DroneSimulation::INPUT_SIZE)
    {
        spdlog::error("DroneSimulation: invalid input size: {}", u.size());
        return;
    }

    // --- 1. INPUT INTERPRETATION ---
    // u is physical: thrust [N], roll/pitch angle setpoints [rad], yaw rate
    // setpoint [rad/s] -- the same command an angle-mode flight controller takes.
    const float thrust = u[0];
    const float roll_sp = u[1];
    const float pitch_sp = u[2];
    const float yaw_rate_sp = u[3];

    // --- 2. ATTITUDE CONTROL (OUTER LOOP) ---
    // Roll and pitch arrive as angles, so an outer loop turns angle error into a
    // rate setpoint for the rate loop below. Yaw arrives as a rate and enters
    // that loop directly. Handing an angle to a rate loop would make the
    // attitude integrate away instead of settling at the commanded tilt.
    const float k_angle = 6.0f;     // angle error -> rate setpoint [1/s]
    const float max_rate = 4.0f;    // rate setpoint saturation [rad/s]

    const Eigen::Vector3f target_rates{
        std::clamp(k_angle * (roll_sp - m_DroneState.roll), -max_rate, max_rate),
        std::clamp(k_angle * (pitch_sp - m_DroneState.pitch), -max_rate, max_rate),
        yaw_rate_sp
    };

    // --- 3. ATTITUDE CONTROL (RATE LOOP) ---
    // These gains define how quickly the drone reaches the commanded rate
    const float k_roll = 0.1f;
    const float k_pitch = 0.1f;
    const float k_yaw = 0.05f;

    Eigen::Vector3f current_rates{ m_DroneState.roll_vel, m_DroneState.pitch_vel, m_DroneState.yaw_vel };

    // Calculate Torques based on rate error
    float roll_torque = k_roll * (target_rates.x() - current_rates.x());
    float pitch_torque = k_pitch * (target_rates.y() - current_rates.y());
    float yaw_torque = k_yaw * (target_rates.z() - current_rates.z());

    // Angular Acceleration (Euler's Equations of motion for a rigid body)
    const Eigen::Vector3f att_acc{
        (m_Model.m_Iyy - m_Model.m_Izz) / m_Model.m_Ixx * m_DroneState.pitch_vel * m_DroneState.yaw_vel + (1 / m_Model.m_Ixx) * roll_torque,
        (m_Model.m_Izz - m_Model.m_Ixx) / m_Model.m_Iyy * m_DroneState.yaw_vel * m_DroneState.roll_vel + (1 / m_Model.m_Iyy) * pitch_torque,
        (m_Model.m_Ixx - m_Model.m_Iyy) / m_Model.m_Izz * m_DroneState.roll_vel * m_DroneState.pitch_vel + (1 / m_Model.m_Izz) * yaw_torque
    };

    // Integrate Angular State
    m_DroneState.set_att_vel_vector(m_DroneState.att_vel_vector() + att_acc * dt);
    m_DroneState.set_att_vector(m_DroneState.att_vector() + m_DroneState.att_vel_vector() * dt);

    // --- 4. TRANSLATIONAL DYNAMICS ---

    // A. Create the Full Rotation Matrix (Body -> World)
    // This handles Roll, Pitch, AND Yaw in one transformation
    const CPose R_body_to_world{ 0.F, 0.F, 0.F, m_DroneState.roll, m_DroneState.pitch, m_DroneState.yaw };
    Eigen::Matrix3f R = R_body_to_world.rotation_3x3();

    // B. Transform Thrust from Body Frame to World Frame
    // In Body Frame, thrust is always +Z (up relative to the drone)
    Eigen::Vector3f thrust_body{ 0.F, 0.F, thrust };
    Eigen::Vector3f thrust_world = R * thrust_body;

    // C. Calculate Drag Force (World Frame)
    // Drag acts directly against the current world velocity
    Eigen::Vector3f drag_force_world{
        -m_Model.m_Cdx * m_DroneState.vx,
        -m_Model.m_Cdy * m_DroneState.vy,
        -m_Model.m_Cdz * m_DroneState.vz
    };

    // D. Gravity Force (World Frame)
    Eigen::Vector3f gravity_world{ 0.F, 0.F, -m_Model.m_Mass * GRAVITY };

    // E. Sum of Forces / Mass = Acceleration (World Frame)
    // F_total = F_thrust + F_gravity + F_drag
    Eigen::Vector3f acc_world = (1.F / m_Model.m_Mass) * (thrust_world + gravity_world + drag_force_world);

    // --- 5. STATE UPDATE ---
    m_DroneState.set_acc_vector(acc_world);

    // Update Velocity: V_new = V_old + a * dt
    m_DroneState.set_vel_vector(m_DroneState.vel_vector() + acc_world * dt);

    // Update Position: P_new = P_old + V * dt
    m_DroneState.set_pos_vector(m_DroneState.pos_vector() + m_DroneState.vel_vector() * dt);
}

float ARParrotDroneSim::pid::update(float sp, float y, float dt)
{
    const auto err = sp - y;
    const auto derr = (err - e_prev) / dt;
    e_prev = err;
    integ += err * dt;

    return kp * err + ki * integ + kd * derr;
}

ARParrotDroneSim::ARParrotDroneSim(const DroneModel& _model)
    : DroneSimulation(_model),
    pid_roll{ 0.05F, 0.F, 0.F },
    pid_pitch{ 0.05F, 0.F, 0.F },
    pid_yaw{ 1.F, 0.F, 0.F },
    pid_h{ 0.5F, 0.F, 0.F },
    pid_roll_sp{ 0.2F, 0.F, 0.F },
    pid_pitch_sp{ 0.2F, 0.F, 0.F }
{
    const float hover_angular_speed = m_HoverThrottle * m_MaxRPM * 2.F * PI / 60.F;
    m_b = (GRAVITY * m_Model.m_Mass) / (4.F * hover_angular_speed * hover_angular_speed);
}

void ARParrotDroneSim::step(const Eigen::VectorXf& u, float dt)
{
    if (u.size() != ARParrotDroneSim::INPUT_SIZE)
    {
        spdlog::error("ARParrotDroneSim: invalid input size {}", u.size());
        return;
    }

    // Control input
    const float vx = u[0] * 1.25F;
    const float vy = u[1] * 1.175F;
    const float vz = u[2] * 1.37484162F;
    const float vr = u[3] * 0.41016432F;

    // OLD
    const CPose R{ 0.F, 0.F, 0.F, 0.F, 0.F, -m_DroneState.yaw };

    const Eigen::Vector3f vel_w{ m_DroneState.vx, m_DroneState.vy, 0.F };
    const Eigen::Vector3f acc_w{ m_DroneState.ax, m_DroneState.ay, 0.F };

    const Eigen::Vector3f vel = R.rotation_3x3() * vel_w;
    const Eigen::Vector3f acc = R.rotation_3x3() * acc_w;

    // the real drone decelerates faster than it accelerates:
    //  0m/s -> 0.5m/s takes about 25s
    //  0.5m/s -> 0m/s takes about 5s
    const float x_acc_coeff = (fabsf(vx) < fabsf(vel.x())) ? 2.F : 7.F;
    const float y_acc_coeff = (fabsf(vy) < fabsf(vel.y())) ? 2.F : 7.F;

    const float roll_sp = pid_roll_sp.update((vy - vel.y()) / y_acc_coeff, acc.y(), dt);
    const float pitch_sp = pid_pitch_sp.update((vx - vel.x()) / x_acc_coeff, acc.x(), dt);

    const float adj_roll = pid_roll.update(roll_sp, -m_DroneState.roll_vel, dt);
    const float adj_pitch = pid_pitch.update(pitch_sp, m_DroneState.pitch_vel, dt);
    const float adj_yaw = pid_yaw.update(vr, m_DroneState.yaw_vel, dt);
    const float adj_h = pid_h.update(vz, m_DroneState.vz, dt);

    // NEW
    // Constants for real drone, determined experimentally
    //Eigen::VectorXf pitch_coeffs(5);
    //pitch_coeffs << 1.964e-18F, -0.029895F, -2.2877e-17F, -0.016256F, 2.827e-17F;
    //Eigen::VectorXf roll_coeffs(5);
    //roll_coeffs << 2.8224e-18F, -0.043291F, -3.2767e-17F, -0.0010363F, 4.2405e-17F;

    //const float target_pitch = CPolynomialFitting::polyeval(pitch_coeffs, -vx);
    //const float target_roll = CPolynomialFitting::polyeval(roll_coeffs, vy);
    //const float target_vz = vz * 1.37484162F;
    //const float target_vr = vr * 0.41016432F;

    //const float adj_roll = pid_roll.update((fabs(target_roll) < 1e-6) ? 0.F : target_roll, roll, dt);
    //const float adj_pitch = pid_pitch.update((fabs(target_pitch) < 1e-6) ? 0.F : target_pitch, pitch, dt);
    ////const float adj_pitch = target_pitch * 0.001F;
    //const float adj_yaw = pid_yaw.update(target_vr, yaw_vel, dt);
    //const float adj_h = pid_h.update(target_vz, z_vel, dt);

    float throttle = m_HoverThrottle + adj_h;
    if (throttle < m_MinThrottle) throttle = m_MinThrottle;
    if (throttle > m_MaxThrottle) throttle = m_MaxThrottle;

    // m0 = front, left
    // m1 = front, right
    // m2 = back, right
    // m3 = back, left

    const Eigen::Vector4f T{ // throttle for each motor
        throttle + adj_roll - adj_pitch - adj_yaw,
        throttle - adj_roll - adj_pitch + adj_yaw,
        throttle - adj_roll + adj_pitch - adj_yaw,
        throttle + adj_roll + adj_pitch + adj_yaw
    };

    const Eigen::Vector4f w = T * m_MaxRPM * 2.F * PI / 60.F; // angular speed of each motor (rad/s)

    const float w_mean_f = (w[0] + w[1]) * 0.5F; // front
    const float w_mean_b = (w[2] + w[3]) * 0.5F; // back
    const float w_mean_l = (w[0] + w[3]) * 0.5F; // left
    const float w_mean_r = (w[1] + w[2]) * 0.5F; // right

    const float thrust = m_b * (powf(w[0], 2) + powf(w[1], 2) + powf(w[2], 2) + powf(w[3], 2));
    const float pitch_torque = m_b * m_l * (powf(w_mean_b, 2) - powf(w_mean_f, 2));
    const float roll_torque = m_b * m_l * (powf(w_mean_r, 2) - powf(w_mean_l, 2));
    const float yaw_torque = m_d * (powf(w[1], 2) + powf(w[3], 2) - powf(w[0], 2) - powf(w[2], 2));

    Eigen::VectorXf u_sim(4);
    u_sim << thrust, roll_torque, pitch_torque, yaw_torque;

    // TODO: this hands body torques [Nm] to a step() that reads its last three
    // slots as roll/pitch angle and yaw rate setpoints, so the attitude channel
    // is misinterpreted (it was, equally, when step() read them as body rates).
    // Only the 'parrot' simulator type reaches here. Fixing it means splitting
    // the attitude and translational halves of DroneSimulation::step, so that a
    // model which already produces torques can skip the attitude loop.
    DroneSimulation::step(u_sim, dt);
}


void AdvancedDroneInput::from_vector(const Eigen::VectorXf& u)
{
    mode = Mode::none;
    if (u.size() == 0)
    {
        spdlog::error("AdvancedDroneInput: Control vector can't be empty.");
        return;
    }

    mode = static_cast<Mode>(static_cast<int>(u[0]));
    if (mode == Mode::none)
    {
        // do nothing
    }
    else if (mode == Mode::body_torques)
    {
        if (u.size() != BODY_TORQUE_SIZE)
        {
            spdlog::error("AdvancedDroneInput: control vector should contain [mode, 3 body torques, collective thrust]");
            return;
        }

        body_torques = u.segment(1, 3);
        collective_thrust = u[4];
    }
    else
    {
        if (u.size() != SIZE)
        {
            spdlog::error("AdvancedDroneInput: control vector should contain {} values", SIZE);
            return;
        }

        orientation = u.segment<4>(1);
        bodyrates = u.segment(5, 3);
        angular_accelerations = u.segment(8, 3);
        collective_thrust = u[11];
    }
}

Eigen::VectorXf AdvancedDroneInput::to_vector() const
{
    Eigen::VectorXf u;
    if (mode == Mode::none)
    {
        u = Eigen::VectorXf::Zero(1);
        u << static_cast<float>(mode);
    }
    else if (mode == Mode::body_torques)
    {
        u = Eigen::VectorXf::Zero(BODY_TORQUE_SIZE);
        u << static_cast<float>(mode), body_torques, collective_thrust;
    }
    else
    {
        u = Eigen::VectorXf::Zero(SIZE);
        u << static_cast<float>(mode), orientation.coeffs(), bodyrates, angular_accelerations, collective_thrust;
    }

    return u;
}

void AdvancedDroneSimulation::step(const Eigen::VectorXf& u, float dt)
{
    AdvancedDroneInput input;
    input.from_vector(u);

    switch (input.mode)
    {
    case AdvancedDroneInput::Mode::attitude:
    {
        const Eigen::Vector3f rate_cmd = attitudeControl(input);
        input.bodyrates = rate_cmd;

        const Eigen::Vector4f _u = bodyRateControl(input);
        DroneSimulation::step(_u, dt);

        body_torques_est = _u.tail<3>(); // TODO?
    }
    break;
    case AdvancedDroneInput::Mode::body_rates:
    {
        const Eigen::Vector4f _u = bodyRateControl(input);
        DroneSimulation::step(_u, dt);

        body_torques_est = _u.tail<3>(); // TODO?
    }
    break;
    case AdvancedDroneInput::Mode::angular_accelerations:
    {
        Eigen::Matrix3f inertia = Eigen::Matrix3f::Zero();
        inertia.diagonal() << m_Model.m_Ixx, m_Model.m_Iyy, m_Model.m_Izz;

        Eigen::Vector4f _u = Eigen::Vector4f::Zero();
        _u[0] = input.collective_thrust;
        _u.tail<3>() = inertia * input.angular_accelerations +
            input.bodyrates.cross(inertia * input.bodyrates);

        DroneSimulation::step(_u, dt);

        body_torques_est = _u.tail<3>(); // TODO?
    }
    break;
    case AdvancedDroneInput::Mode::body_torques:
    {
        Eigen::Vector4f _u = Eigen::Vector4f::Zero();
        _u[0] = input.collective_thrust;
        _u.tail<3>() = input.body_torques;

        DroneSimulation::step(_u, dt);

        body_torques_est = _u.tail<3>(); // TODO?
    }
    break;
    case AdvancedDroneInput::Mode::none:
    default:
        break;
    }
}

Eigen::Vector3f AdvancedDroneSimulation::attitudeControl(const AdvancedDroneInput& cmd)
{
    // roll and pitch attitude control gain
    const float roll_pitch_cont_gain = 6.f;
    const float yaw_cont_gain = 0.f;

    const Eigen::VectorXf att = m_DroneState.att_vector();

    const CQuaternion _q_att{ att.x(), att.y(), att.z() };
    const Eigen::Quaternionf q_att{ _q_att.w(), _q_att.x(), _q_att.y(), _q_att.z() };
    const Eigen::Quaternionf q_error = q_att.inverse() * cmd.orientation;

    const float sign = (q_error.w() >= 0) ? 1.f : -1.f;

    return { // w_fb
        cmd.bodyrates.x() + sign * 2.f * roll_pitch_cont_gain * q_error.x(),
        cmd.bodyrates.y() + sign * 2.f * roll_pitch_cont_gain * q_error.y(),
        cmd.bodyrates.z() + sign * 2.f * yaw_cont_gain * q_error.z()
    };
}

Eigen::Vector4f AdvancedDroneSimulation::bodyRateControl(const AdvancedDroneInput& cmd)
{
    const Eigen::Vector3f bodyrates = cmd.bodyrates;
    const Eigen::Vector3f angular_accelerations = cmd.angular_accelerations;
    const float collective_thrust = cmd.collective_thrust;

    const float body_rates_p_xy = 0.15F;
    const float body_rates_p_z = 0.5F;
    const float body_rates_d_xy = 0.03F;
    const float body_rates_d_z = 0.1F;

    Eigen::MatrixXf K_lqr = Eigen::MatrixXf::Zero(3, 6);
    K_lqr(0, 0) = body_rates_p_xy;
    K_lqr(1, 1) = body_rates_p_xy;
    K_lqr(2, 2) = body_rates_p_z;
    K_lqr(0, 3) = body_rates_d_xy;
    K_lqr(1, 4) = body_rates_d_xy;
    K_lqr(2, 5) = body_rates_d_z;

    Eigen::VectorXf control_error = Eigen::VectorXf::Zero(6);

    Eigen::Matrix3f inertia = Eigen::Matrix3f::Zero();
    inertia.diagonal() << m_Model.m_Ixx, m_Model.m_Iyy, m_Model.m_Izz;

    const Eigen::Vector3f body_rate_estimate = m_DroneState.att_vel_vector();

    control_error.segment(0, 3) = bodyrates - body_rate_estimate;
    control_error.segment(3, 3) =
        bodyrates.cross(inertia * bodyrates) +
        inertia * angular_accelerations -
        body_torques_est;

    Eigen::Vector4f torques_and_thrust{};
    torques_and_thrust[0] = collective_thrust;

    torques_and_thrust.tail<3>() =
        K_lqr * control_error +
        body_rate_estimate.cross(inertia * body_rate_estimate) +
        inertia * angular_accelerations;

    return torques_and_thrust;
}
