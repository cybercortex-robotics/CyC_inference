// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CModelLegged_H_
#define CModelLegged_H_

#include <CyC_TYPES.h>
#include <iostream>
#include "CDenavitHartenberg.h"
#include "CBaseStateSpaceModel.h"
#include "CLinearFirstOrderSystem.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"
#include "os/CTimer.h"

static const size_t ModelLegged_NumLegs = 4; // number of legs
static const size_t ModelLegged_NumJoints = 3; // joints per leg
static const size_t ModelLegged_JointStates = 3; // variables per joint (q, dq, ddq)
static const size_t ModelLegged_BodyPositionStates = 3; // body translation
static const size_t ModelLegged_BodyAttitudeStates = 4; // body rotation as quaternion

static const size_t ModelLegged_NumInputs = ModelLegged_NumLegs * ModelLegged_NumJoints;
static const size_t ModelLegged_NumJointStates = ModelLegged_NumInputs * ModelLegged_JointStates;
static const size_t ModelLegged_NumBodyStates = ModelLegged_BodyPositionStates + ModelLegged_BodyAttitudeStates;
static const size_t ModelLegged_NumStates = ModelLegged_NumJointStates + ModelLegged_NumBodyStates; // joints + body pose
static const size_t ModelLegged_NumOutputs = 5;


enum class FOOT_INDEX
{
    FOOT_FL = 0,
    FOOT_FR = 1,
    FOOT_RL = 2,
    FOOT_RR = 3,
    NUM
};

enum class SIM_TYPE
{
    KINEMATICS,
    DYNAMICS
};

struct ModelLeggedInput
{
    // structure of "u":
    // [0] = ctrl type
    // [1] = controller state (body, fr, fl, rr, rl)
    // [2-14] = desired angular position
    // [14-26] = desired angular velocity
    // [26-38] = desired angular acceleration
    // [38-50] = desired torque

    enum class Index
    {
        CTRL_TYPE = 0,
        CTRL_STATE = 1,
        POSITION = 2,
        VELOCITY = POSITION + ModelLegged_NumInputs,
        ACCEL = VELOCITY + ModelLegged_NumInputs,
        TORQUE = ACCEL + ModelLegged_NumInputs,

        SIZE = TORQUE + ModelLegged_NumInputs
    };

    ModelLeggedInput() = default;
    explicit ModelLeggedInput(const CycControlInput& input);
    ModelLeggedInput(const ModelLeggedInput&) = default;
    ModelLeggedInput(ModelLeggedInput&&) = default;
    ModelLeggedInput& operator=(const ModelLeggedInput&) = default;
    ModelLeggedInput& operator=(ModelLeggedInput&&) = default;
    ~ModelLeggedInput() = default;

    void setControlType(size_t type);
    size_t getControlType() const;

    void setControllerState(size_t state);
    size_t getControllerState() const;

    bool setDesiredPosition(const Eigen::VectorXf& pos);
    Eigen::Ref<const Eigen::VectorXf> getDesiredPosition() const;

    bool setDesiredVelocity(const Eigen::VectorXf& vel);
    Eigen::Ref<const Eigen::VectorXf> getDesiredVelocity() const;

    bool setDesiredAcceleration(const Eigen::VectorXf& acc);
    Eigen::Ref<const Eigen::VectorXf> getDesiredAcceleration() const;

    bool setDesiredTorque(const Eigen::VectorXf& tau);
    Eigen::Ref<const Eigen::VectorXf> getDesiredTorque() const;

    CycControlInput toControlInput() const;
    bool fromControlInput(const CycControlInput& input);

private:
    Eigen::VectorXf u = Eigen::VectorXf::Zero((size_t)Index::SIZE);
};

struct CModelLegged : public CBaseStateSpaceModel
{
public:
    CModelLegged(const std::string& _legged_model_file, SIM_TYPE simType);
    CModelLegged(const CModelLegged&) = default;
    CModelLegged(CModelLegged&&) = default;
    CModelLegged& operator=(const CModelLegged&) = default;
    CModelLegged& operator=(CModelLegged&&) = default;
    ~CModelLegged() = default;

    virtual bool step(const float& _dt, const Eigen::VectorXf& _u) override;
    bool step(const ModelLeggedInput& input);

    Eigen::VectorXf dh2state() const;
    void dh2state(Eigen::VectorXf& _out_x) const;
    static void dh2state(const CDhRobotModel& _out_dh_model, Eigen::VectorXf& _out_x);

    void state2dh(CDhRobotModel& _out_dh_model) const;
    static void state2dh(const Eigen::VectorXf& _x, CDhRobotModel& _out_dh_model);

    Eigen::VectorXf getMotorAngles() const;
    Eigen::VectorXf getMotorVelocities() const;
    Eigen::VectorXf getMotorAccelerations() const;
    bool setMotorAngles(const Eigen::VectorXf& _u);

    CDhRobotModel* getDH() const { return m_pDhRobotModel.get(); };

public: // inverse kinematics
    // TODO: add check for ikine when it cannot return a feasible configuration
    // Current solution: result = ikine(...); if (!result.hasNaN()) { ... }
    Eigen::VectorXf ikine(
        const Eigen::Vector3f& _desired_foot_pos_fr,
        const Eigen::Vector3f& _desired_foot_pos_fl,
        const Eigen::Vector3f& _desired_foot_pos_rr,
        const Eigen::Vector3f& _desired_foot_pos_rl,
        const CPose& _body_pose) const;

    Eigen::Vector3f ikine_f(FOOT_INDEX idx, const Eigen::Vector3f& pstar, float tx, float ty, float tz, float Rx, float Ry, float Rz) const;

    bool step_dynamics(const float dt, const bool isMoving, const Eigen::VectorXf& _u);
    bool step_kinematics(const float dt, const bool isMoving, const Eigen::VectorXf& _u);

private:
    bool isMovingBody(const Eigen::VectorXf& u) const;

public:
    static float angular_diff(float theta1, float theta2);

    SIM_TYPE m_simType = SIM_TYPE::KINEMATICS;
    std::unique_ptr<CDhRobotModel> m_pDhRobotModel;
    std::vector<CLinearFirstOrderSystem> m_JointMotorModels;

    Eigen::VectorXf m_q = Eigen::VectorXf::Zero(ModelLegged_NumInputs);
    Eigen::VectorXf m_qdot = Eigen::VectorXf::Zero(ModelLegged_NumInputs);
    Eigen::VectorXf m_qddot = Eigen::VectorXf::Zero(ModelLegged_NumInputs);

    CTimer m_simTimer;
};

#endif /* CModelLegged_H_ */
