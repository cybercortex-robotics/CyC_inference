#include "CModelVehicle.h"

CModelVehicle::CModelVehicle(const VehicleType _vehicle_type,
    const float _vehicle_length,
    const float _vehicle_width,
    const float _full_vehicle_length,
    const float _full_vehicle_width,
    const float _wheel_radius,
    const float _max_steering_angle,
    const float _max_steering_rate,
    const float _max_forward_speed,
    const float _max_reverse_speed,
    const float _max_acceleration,
    const CyC_INT _pulses_per_rotation_left,
    const CyC_INT _pulses_per_rotation_right) :
    CBaseStateSpaceModel(ModelVehicle_NumStates, ModelVehicle_NumInputs, ModelVehicle_NumOutputs)
{
    m_VehicleType = _vehicle_type;
    m_fLongDistWheels = _vehicle_length;
    m_fLatDistWheels = _vehicle_width;
    m_fVehicleLength = _full_vehicle_length;
    m_fVehicleWidth = _full_vehicle_width;
    m_fWheelRadius = _wheel_radius;
    m_fWheelCircumference = 2.f * PI * m_fWheelRadius;
    m_fMaxSteeringAngleRad = _max_steering_angle;
    m_fSteeringRateResolution = _max_steering_rate;
    m_fMaxForwardSpeed = _max_forward_speed;
    m_fMaxReverseSpeed = _max_reverse_speed;
    m_fMaxAcceleration = _max_acceleration;
    m_iPulsesPerRotationLeft = _pulses_per_rotation_left;
    m_iPulsesPerRotationRight = _pulses_per_rotation_right;

    init();
}

CModelVehicle::CModelVehicle(const std::string& _vehicle_model_file) :
    CBaseStateSpaceModel(ModelVehicle_NumStates, ModelVehicle_NumInputs, ModelVehicle_NumOutputs)
{
    if (!fs::exists(_vehicle_model_file.c_str()))
    {
       spdlog::error("{}: Vehicle model file does not exist.", typeid(*this).name());
       return;
    }

    libconfig::Config configFile;
    configFile.readFile(_vehicle_model_file.c_str());

    std::string str;
    configFile.lookupValue("type", str);
    if (str.compare("ackermann") == 0)
    {
        m_VehicleType = VehicleType_Ackermann;
    }
    else if (str.compare("differential") == 0)
    {
        m_VehicleType = VehicleType_Differential;
    }
    else
    {
        spdlog::error("{}: Vehicle type '{}' unknown.", typeid(*this).name(), str);
        m_bInitialized = false;
    }

    configFile.lookupValue("long_dist_wheels", m_fLongDistWheels);
    configFile.lookupValue("lat_dist_wheels", m_fLatDistWheels);
    configFile.lookupValue("vehicle_length", m_fVehicleLength);
    configFile.lookupValue("vehicle_width", m_fVehicleWidth);
    configFile.lookupValue("wheel_radius", m_fWheelRadius);
    m_fWheelCircumference = 2.f * PI * m_fWheelRadius;
    configFile.lookupValue("max_steering_angle", m_fMaxSteeringAngleRad);
    configFile.lookupValue("steering_rate_resolution", m_fSteeringRateResolution);
    configFile.lookupValue("max_forward_speed", m_fMaxForwardSpeed);
    configFile.lookupValue("max_reverse_speed", m_fMaxReverseSpeed);
    configFile.lookupValue("max_acceleration", m_fMaxAcceleration);
    configFile.lookupValue("pulses_per_rotation_left", m_iPulsesPerRotationLeft);
    configFile.lookupValue("pulses_per_rotation_right", m_iPulsesPerRotationRight);

    init();

    m_bInitialized = true;
}

void CModelVehicle::init()
{
    velocity_sim.setConstants(1.f, 0.f);
}

bool CModelVehicle::step(const float& _dt, const Eigen::VectorXf& _u)
{
    // u[0] - linear velocity (throtle)
    // u[1] - steering angle or steering angular velocity TODO: check

    if (_u.size() != m_nNumInputs)
    {
        spdlog::error("{}: Control input size different from vehicle model. Could not perform simulation step.", typeid(*this).name());
        return false;
    }

    if (m_VehicleType == VehicleType_Ackermann)
    {
        step_ackermann(_dt, _u);
    }
    else if (m_VehicleType == VehicleType_Differential)
    {
        step_differential(_dt, _u);
    }
    else
    {
        spdlog::error("{}: Vehicle type '{}' unknown.", typeid(*this).name(), static_cast<int>(m_VehicleType));
        return false;
    }

    // Add velocity actuator constraints
    if (m_x(2) > m_fMaxForwardSpeed)
        m_x(2) = m_fMaxForwardSpeed;
    else if (m_x(2) < -m_fMaxReverseSpeed)
        m_x(2) = -m_fMaxReverseSpeed;

    // Update the measurements
    //y = model.C * x;
    m_y = m_x;

    return true;
}

void CModelVehicle::step_ackermann(const float& _dt, const Eigen::VectorXf& _u)
{
    const float side_slip = atanf(tanf(_u(1)) / 2.f);

    m_x(4) = _u(1);    // Steering angle

    //x(2) += u(0) * dt;                    // acceleration
    m_x(2) = velocity_sim.step(_u(0), _dt);  // velocity

    // heading
    //m_x(3) += (m_x(2) / m_fLongDistWheels) * tanf(_u(1)) * _dt;
    m_x(3) += ((m_x(2) * cosf(side_slip) * tanf(_u(1))) / m_fLongDistWheels) * _dt;

    m_x(0) += m_x(2) * cosf(m_x(3) + side_slip) * _dt;  // x position
    m_x(1) += m_x(2) * sinf(m_x(3) + side_slip) * _dt;  // y position
}

void CModelVehicle::step_differential(const float& _dt, const Eigen::VectorXf& _u)
{
    float v = 0.F;
    float w = 0.F;
    if (abs(_u(1)) < 0.0001F) // moving in a straight line
    {
        v = _u(0);
        w = _u(1);
    }
    else // moving on a curve
    {
        const float r = _u(0) / _u(1);
        const float vr = _u(1) * (r + 0.5F * this->m_fLatDistWheels);
        const float vl = _u(1) * (r - 0.5F * this->m_fLatDistWheels);

        v = (vr + vl) * 0.5F;
        w = (vr - vl) / this->m_fLatDistWheels;
    }

    m_x(0) += m_x(2) * cosf(m_x(3)) * _dt;
    m_x(1) += m_x(2) * sinf(m_x(3)) * _dt;
    m_x(2) = velocity_sim.step(v, _dt);
    m_x(3) += w * _dt;
}

void CModelVehicle::linearizeMethod1(const float &velocity, const float &yaw_angle, const float &ref_steer_angle, const float &dt)
{
    // Linearize the system matrix
    m_A << 1.,    0.,     cos(yaw_angle)*dt,                              -velocity * sin(yaw_angle)*dt,
           0,     1.,     sin(yaw_angle)*dt,                               velocity * cos(yaw_angle)*dt,
           0,     0,      1.,                                              0,
           0,     0,      (tan(ref_steer_angle) / m_fLongDistWheels)*dt,   1.;

    // Linearize the input matrix
    m_B << 0.,    0.,
           0.,    0.,
           dt,    0.,
           0.,    velocity / (cos(ref_steer_angle) * cos(ref_steer_angle) * m_fLongDistWheels);

    // Linearization constant
    m_E << velocity * sin(yaw_angle) * yaw_angle * dt,
          -velocity * cos(yaw_angle) * yaw_angle * dt,
           0.,
          -(velocity * ref_steer_angle / (m_fLongDistWheels * cos(ref_steer_angle) * cos(ref_steer_angle))) * dt;
}

void CModelVehicle::linearizeMethod2(const float velocity, const float yaw_angle, const float ref_steer_angle, const float dt)
{
    const float lr = m_fLongDistWheels / 2.f; // center of gravity at the center of vehicle
    const float lf = m_fLongDistWheels / 2.f; // center of gravity at the center of vehicle

    const float tans = tan(ref_steer_angle);
    const float beta = atanf(tans * lr / m_fLongDistWheels);

    const float common1 = velocity * lr * powf(1.f / cosf(ref_steer_angle), 2.f);
    const float common2 = lr * lr * tans * tans / powf(m_fLongDistWheels, 2.f) + 1.f;

    const float dx_dv = cosf(yaw_angle + beta);
    const float dx_dw = -velocity * sinf(yaw_angle + beta);
    const float dx_du2_num = -common1 * sinf(yaw_angle + beta);
    const float dx_du2_den = m_fLongDistWheels * common2;

    const float dy_dv = sinf(yaw_angle + beta);
    const float dy_dw = velocity * cosf(yaw_angle + beta);
    const float dy_du2_num = common1 * cosf(yaw_angle + beta);
    const float dy_du2_den = dx_du2_den; // same denominator

    const float dw_dv = tans / (m_fLongDistWheels * sqrtf(common2));
    const float dw_du2_num = velocity * powf(1.f / cos(ref_steer_angle), 2.f);
    const float dw_du2_den = m_fLongDistWheels * sqrtf(powf(common2, 3.f));

    // Linearize the system matrix
    m_A << 0.f, 0.f, dx_dv, dx_dw,
           0.f, 0.f, dy_dv, dy_dw,
           0.f, 0.f, 0.f,   0.f,
           0.f, 0.f, dw_dv, 0.f;

    // Linearize the input matrix
    m_B << 0.f, dx_du2_num / dx_du2_den,
           0.f, dy_du2_num / dy_du2_den,
           1.f, 0.f,
           0.f, dw_du2_num / dw_du2_den;

    // Linearization constant
    // E << velocity * sin(yaw_angle) * yaw_angle * dt,
    //     -velocity * cos(yaw_angle) * yaw_angle * dt,
    //     0.,
    //     -(velocity * ref_steer_angle / (m_fVehicleLength * cos(ref_steer_angle) * cos(ref_steer_angle))) * dt;
}
