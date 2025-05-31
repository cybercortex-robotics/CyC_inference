// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSTATESPACESIMULATIONDRONE_H_
#define CSTATESPACESIMULATIONDRONE_H_

#include "CyC_TYPES.h"
#include "CStateSpaceModelDrone.h"

class DroneState
{
public:
    enum EStateIndex
    {
        POS_X, POS_Y, POS_Z,
        VEL_X, VEL_Y, VEL_Z,
        ACC_X, ACC_Y, ACC_Z,
        ROLL, PITCH, YAW,
        VEL_ROLL, VEL_PITCH, VEL_YAW,

        STATE_NUM,

        STATE_DIMENSIONS = 3, // e.g.: 2D, 3D, etc.

        POS_BEGIN = POS_X,
        VEL_BEGIN = VEL_X,
        ACC_BEGIN = ACC_X,
        ATT_BEGIN = ROLL,
        ATT_VEL_BEGIN = VEL_ROLL,
    };

    DroneState();
    DroneState(Eigen::VectorXf x);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    bool from_vector(const Eigen::VectorXf& x);

    const Eigen::VectorXf& as_vector() const { return state; }

    // position
    const Eigen::VectorXf pos_vector() const { return get_partial_state(POS_BEGIN); }
    bool set_pos_vector(const Eigen::VectorXf& pos) { return set_partial_state(POS_BEGIN, pos); }

    // velocity
    const Eigen::VectorXf vel_vector() const { return get_partial_state(VEL_BEGIN); }
    bool set_vel_vector(const Eigen::VectorXf& vel) { return set_partial_state(VEL_BEGIN, vel); }

    // acceleration
    const Eigen::VectorXf acc_vector() const { return get_partial_state(ACC_BEGIN); }
    bool set_acc_vector(const Eigen::VectorXf& acc) { return set_partial_state(ACC_BEGIN, acc); }

    // attitude (rpy)
    const Eigen::VectorXf att_vector() const { return get_partial_state(ATT_BEGIN); }
    bool set_att_vector(const Eigen::VectorXf& att) { return set_partial_state(ATT_BEGIN, att); }

    // attitude velocity (rpy velocity)
    const Eigen::VectorXf att_vel_vector() const { return get_partial_state(ATT_VEL_BEGIN); }
    bool set_att_vel_vector(const Eigen::VectorXf& att_vel) { return set_partial_state(ATT_VEL_BEGIN, att_vel); }

private:
    const Eigen::VectorXf get_partial_state(Eigen::Index begin) const;
    bool set_partial_state(Eigen::Index begin, const Eigen::VectorXf& x);

    Eigen::VectorXf state; // MUST be the first member!!

public:
    float& x = state[POS_X];
    float& y = state[POS_Y];
    float& z = state[POS_Z];
    float& vx = state[VEL_X];
    float& vy = state[VEL_Y];
    float& vz = state[VEL_Z];
    float& ax = state[ACC_X];
    float& ay = state[ACC_Y];
    float& az = state[ACC_Z];
    float& roll = state[ROLL];
    float& pitch = state[PITCH];
    float& yaw = state[YAW];
    float& roll_vel = state[VEL_ROLL];
    float& pitch_vel = state[VEL_PITCH];
    float& yaw_vel = state[VEL_YAW];
};

class DroneSimulation
{
public:
    constexpr static size_t INPUT_SIZE = 4;

    const DroneModel& model;
    DroneState state;

    DroneSimulation(const DroneModel& _model) :
        model(_model)
    {
    }

    virtual void step(const Eigen::VectorXf& u, float dt);
};

class ARParrotDroneSim : public DroneSimulation
{
public:
    constexpr static size_t INPUT_SIZE = 4;

    ARParrotDroneSim(const DroneModel& _model);

    void step(const Eigen::VectorXf& u, float dt) override;

private:
    struct pid
    {
        float kp;
        float ki;
        float kd;

        float integ;
        float e_prev;

        pid(float _kp, float _ki, float _kd)
            : kp(_kp), ki(_ki), kd(_kd)
        {
            reset();
        }

        float update(float sp, float y, float dt);

        void reset()
        {
            integ = 0.F;
            e_prev = 0.F;
        }
    };

    pid pid_roll;
    pid pid_pitch;
    pid pid_yaw;
    pid pid_h;

    pid pid_roll_sp;
    pid pid_pitch_sp;

    // TODO: Add some(?) to the model
    const CyC_INT m_MaxRPM = 28500;
    const float m_MinThrottle = 0.5F;
    const float m_MaxThrottle = 0.85F;
    const float m_HoverThrottle = 0.66F;
    const float m_MaxPitchRoll = 12.F * DEG2RAD;

    float m_b = 0.F; // thrust factor ~6.3208311491351028406989660778488e-7
    const float m_d = 6.0e-10F; // drag factor TODO!!!
    const float m_l = 0.1F; // distance from body center to rotor center

    friend class CDwaDroneController;
};

class AdvancedDroneInput
{
public:
    AdvancedDroneInput() = default;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    enum class Mode
    {
        none,
        attitude,
        body_rates,
        angular_accelerations,
        body_torques
    };

    // mode + orientation + bodyrates + angular_vel + collective_thrust
    static constexpr size_t SIZE = 1 + 4 + 3 + 3 + 1;
    // mode + 3 body torques + collective_thrust
    static constexpr size_t BODY_TORQUE_SIZE = 5;

    Mode mode = Mode::none;

    // Desired orientation of the body frame with respect to the world frame
    Eigen::Quaternionf orientation;

    // Desired body frames rad/s
    Eigen::Vector3f bodyrates = Eigen::Vector3f::Zero();

    // Desired angular accelerations of body frame rad/s^2
    Eigen::Vector3f angular_accelerations = Eigen::Vector3f::Zero();

    // Collective mass normalized thrust m/s^2
    float collective_thrust = 0.f;

    // Body torques Nm
    Eigen::VectorXf body_torques = Eigen::VectorXf::Zero(3);

    void from_vector(const Eigen::VectorXf& u);
    Eigen::VectorXf to_vector() const;
};

class AdvancedDroneSimulation : public DroneSimulation
{
public:
    AdvancedDroneSimulation(const DroneModel& _model)
        : DroneSimulation(_model)
    {
    }

    void step(const Eigen::VectorXf& u, float dt) override;

private:
    Eigen::Vector3f attitudeControl(const AdvancedDroneInput& cmd);
    Eigen::Vector4f bodyRateControl(const AdvancedDroneInput& cmd);

    Eigen::Vector3f body_torques_est;
};

#endif // CSTATESPACESIMULATIONDRONE_H_
