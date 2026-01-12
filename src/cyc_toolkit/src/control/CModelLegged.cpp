#include "CModelLegged.h"
#include "CModelLeggedDynamics.h"

CModelLegged::CModelLegged(const std::string& _legged_model_file, SIM_TYPE simType) :
    CBaseStateSpaceModel(ModelLegged_NumStates, ModelLegged_NumInputs, ModelLegged_NumOutputs),
    m_simType(simType)
{
    if (!fs::exists(_legged_model_file.c_str()))
    {
       spdlog::error("{}: Legged robot model file does not exist.", typeid(*this).name());
       return;
    }

    // DH formalism
    m_pDhRobotModel = std::make_unique<CDhRobotModel>(_legged_model_file);

    // Initial state
    m_q = getMotorAngles();
    m_JointMotorModels.resize(m_q.size());
    for (size_t i = 0; i < m_JointMotorModels.size(); i++)
    {
        m_JointMotorModels[i].setConstants(0.01f, m_q(i));
    }

    dh2state(m_x);
    m_bInitialized = true;
}

float CModelLegged::angular_diff(float theta1, float theta2)
{
    return fmodf((theta1 - theta2 + PI), (2.f * PI)) - PI;
}

bool CModelLegged::step(const float& _dt, const Eigen::VectorXf& _u)
{
    if (this->getNumInputs() != _u.size())
    {
        spdlog::error("{}: control input size different from robot model. Could not perform simulation", typeid(*this).name());
        return false;
    }

    const bool isMoving = isMovingBody(_u); // better method?

    if (m_simType == SIM_TYPE::KINEMATICS)
        return step_kinematics(_dt, isMoving, _u);
    else if (m_simType == SIM_TYPE::DYNAMICS)
        return step_dynamics(_dt, isMoving, _u);

    return false;
}

bool CModelLegged::step(const ModelLeggedInput& input)
{
    const bool isMoving = input.getControllerState() == (size_t)FOOT_INDEX::NUM;
    const auto dt = m_simTimer.getElapsedTimeMicroseconds() * MSEC2SEC * MSEC2SEC;
    m_simTimer.restart();

    if (m_simType == SIM_TYPE::KINEMATICS)
        return step_kinematics(dt, isMoving, input.getDesiredPosition());
    else if (m_simType == SIM_TYPE::DYNAMICS)
        return step_dynamics(dt, isMoving, input.getDesiredTorque());

    return false;
}

bool CModelLegged::step_kinematics(const float _dt, bool isMoving, const Eigen::VectorXf& _u)
{
    // Get the foot poses in robot body coordinates system
    std::vector<CPose> end_effectors_b = m_pDhRobotModel->dkine_feet();
    const CPose body_pose_prev = m_pDhRobotModel->getBodyPose();

    const CPose foot_FL_c(body_pose_prev.transform() * end_effectors_b[(size_t)FOOT_INDEX::FOOT_FL].transform());
    const CPose foot_FR_c(body_pose_prev.transform() * end_effectors_b[(size_t)FOOT_INDEX::FOOT_FR].transform());
    const CPose foot_RL_c(body_pose_prev.transform() * end_effectors_b[(size_t)FOOT_INDEX::FOOT_RL].transform());
    const CPose foot_RR_c(body_pose_prev.transform() * end_effectors_b[(size_t)FOOT_INDEX::FOOT_RR].transform());

    const Eigen::Vector3f foot_FL_b = foot_FL_c.translation_3x1();
    const Eigen::Vector3f foot_FR_b = foot_FR_c.translation_3x1();
    const Eigen::Vector3f foot_RL_b = foot_RL_c.translation_3x1();
    const Eigen::Vector3f foot_RR_b = foot_RR_c.translation_3x1();

    // Update joint angles
    for (CyC_UINT i = 0; i < m_pDhRobotModel->m_Arms.size(); ++i)
    {
        std::vector<CLinkDH>* arm = m_pDhRobotModel->m_Arms[i].getArmDH();
        for (CyC_UINT j = 0; j < arm->size(); ++j)
        {
            CyC_UINT u_idx = i * (CyC_UINT)arm->size() + j;

            // float Kp = 2.f;
            // joint_angles = joint_angles + Kp * ang_diff(joint_goal_angles, joint_angles) * dt
            //arm->at(j).theta += Kp * angular_diff(_u[u_idx], arm->at(j).theta) * _dt;
#if 0
            if (!isMoving)
            {
                m_JointMotorModels[u_idx].setConstants(0.01f, arm->at(j).theta);
                arm->at(j).theta = m_JointMotorModels[u_idx].step(_u[u_idx], _dt);
            }
            else
            {
                arm->at(j).theta = _u[u_idx];
            }
#else
            //arm->at(j).theta = m_JointMotorModels[u_idx].step(_u[u_idx], _dt);
            arm->at(j).theta = _u[u_idx];
#endif
    }
}

    // Update body pose
    const CPose body_pose = m_pDhRobotModel->dkine_body(foot_FL_b, foot_FR_b, foot_RL_b, foot_RR_b);
    if (isMoving)
        m_pDhRobotModel->setBodyPose(body_pose);

    // Update state vector
    dh2state(m_x);

    return true;
}

bool CModelLegged::step_dynamics(const float _dt, bool isMoving, const Eigen::VectorXf& _u)
{
    // Update dynamics
    for (CyC_UINT i = 0; i < m_pDhRobotModel->m_Arms.size(); ++i)
    {
        const Eigen::Vector3f q = m_q.segment<ModelLegged_NumJoints>(ModelLegged_NumJoints * i);
        const Eigen::Vector3f qdot = m_qdot.segment<ModelLegged_NumJoints>(ModelLegged_NumJoints * i);
        const bool is_left = (i == (CyC_UINT)FOOT_INDEX::FOOT_FL) || (i == (CyC_UINT)FOOT_INDEX::FOOT_RL);
        const Eigen::Vector3f tau = _u.segment<ModelLegged_NumJoints>(ModelLegged_NumJoints * i);

        if (tau.isZero())
            continue;

        Eigen::Ref<Eigen::VectorXf> qddot = m_qddot.segment<ModelLegged_NumJoints>(ModelLegged_NumJoints * i);
        qddot = forward_dynamics(q, qdot, tau, is_left);

        // simple Euler integration
        Eigen::Ref<Eigen::VectorXf> _q = m_q.segment<ModelLegged_NumJoints>(ModelLegged_NumJoints * i);
        Eigen::Ref<Eigen::VectorXf> _qdot = m_qdot.segment<ModelLegged_NumJoints>(ModelLegged_NumJoints * i);
        _qdot += _dt * qddot;
        _q += _dt * _qdot;
    }

    // Update kinematics
    const bool kresult = step_kinematics(_dt, isMoving, m_q);
    if (kresult)
    {
        for (CyC_UINT i = 0; i < m_pDhRobotModel->m_Arms.size(); ++i)
        {
            std::vector<CLinkDH>& arm = *(m_pDhRobotModel->m_Arms[i].getArmDH());
            for (CyC_UINT j = 0; j < arm.size(); ++j)
            {
                const CyC_UINT idx = i * (CyC_UINT)arm.size() + j;
                arm.at(j).theta_dot = m_qdot[idx]; // Joint angular velocity
                arm.at(j).theta_ddot = m_qddot[idx]; // Joint angular acceleration
            }
        }
    }

    return kresult;
}

Eigen::VectorXf CModelLegged::getMotorAngles() const
{
    Eigen::VectorXf thetas = Eigen::VectorXf(getNumInputs());
    for (CyC_UINT i = 0; i < m_pDhRobotModel->m_Arms.size(); ++i)
    {
        std::vector<CLinkDH>* arm = m_pDhRobotModel->m_Arms[i].getArmDH();

        for (CyC_UINT j = 0; j < arm->size(); ++j)
        {
            const CyC_UINT idx = i * (CyC_UINT)arm->size() + j;
            thetas[idx] = arm->at(j).theta;
        }
    }

    return thetas;
}

Eigen::VectorXf CModelLegged::getMotorVelocities() const
{
    Eigen::VectorXf thetas = Eigen::VectorXf(getNumInputs());
    for (CyC_UINT i = 0; i < m_pDhRobotModel->m_Arms.size(); ++i)
    {
        std::vector<CLinkDH>* arm = m_pDhRobotModel->m_Arms[i].getArmDH();

        for (CyC_UINT j = 0; j < arm->size(); ++j)
        {
            const CyC_UINT idx = i * (CyC_UINT)arm->size() + j;
            thetas[idx] = arm->at(j).theta_dot;
        }
    }

    return thetas;
}

Eigen::VectorXf CModelLegged::getMotorAccelerations() const
{
    Eigen::VectorXf thetas = Eigen::VectorXf(getNumInputs());
    for (CyC_UINT i = 0; i < m_pDhRobotModel->m_Arms.size(); ++i)
    {
        std::vector<CLinkDH>* arm = m_pDhRobotModel->m_Arms[i].getArmDH();

        for (CyC_UINT j = 0; j < arm->size(); ++j)
        {
            const CyC_UINT idx = i * (CyC_UINT)arm->size() + j;
            thetas[idx] = arm->at(j).theta_ddot;
        }
    }

    return thetas;
}

bool CModelLegged::setMotorAngles(const Eigen::VectorXf& _u)
{
    if (this->getNumInputs() != _u.size())
    {
        spdlog::error("{} setMotorAngles: number of input angles is different from the number of motors.", typeid(*this).name());
        return false;
    }

    for (CyC_UINT i = 0; i < m_pDhRobotModel->m_Arms.size(); ++i)
    {
        std::vector<CLinkDH>* arm = m_pDhRobotModel->m_Arms[i].getArmDH();
        for (CyC_UINT j = 0; j < arm->size(); ++j)
        {
            const CyC_UINT idx = i * (CyC_UINT)arm->size() + j;
            arm->at(j).theta = _u[idx];
        }
    }

    return true;
}

void CModelLegged::dh2state(const CDhRobotModel& pDhRobotModel, Eigen::VectorXf& _out_x)
{
    _out_x = Eigen::VectorXf::Zero(ModelLegged_NumStates);
    for (CyC_UINT i = 0; i < pDhRobotModel.m_Arms.size(); ++i)
    {
        const std::vector<CLinkDH>* arm = pDhRobotModel.m_Arms[i].getArmDH();
        for (CyC_UINT j = 0; j < arm->size(); ++j)
        {
            const CyC_UINT idx = i * (CyC_UINT)arm->size() + j;

            _out_x[idx * ModelLegged_JointStates + 0] = arm->at(j).theta; // Joint angle
            _out_x[idx * ModelLegged_JointStates + 1] = arm->at(j).theta_dot; // Joint angular velocity
            _out_x[idx * ModelLegged_JointStates + 2] = arm->at(j).theta_ddot; // Joint angular acceleration
        }
    }

    const Eigen::Vector3f trans = pDhRobotModel.getBodyPose().translation_3x1();
    const Eigen::Vector4f rot = pDhRobotModel.getBodyPose().rotation_quat().to_vector();

    const size_t translationIndex = ModelLegged_NumJointStates;
    const size_t rotationIndex = translationIndex + ModelLegged_BodyPositionStates;

    _out_x.segment<ModelLegged_BodyPositionStates>(translationIndex) = trans;
    _out_x.segment<ModelLegged_BodyAttitudeStates>(rotationIndex) = rot;
}

void CModelLegged::dh2state(Eigen::VectorXf& _out_x) const
{
    dh2state(*m_pDhRobotModel, _out_x);
}

Eigen::VectorXf CModelLegged::dh2state() const
{
    Eigen::VectorXf _out_x;
    dh2state(_out_x);

    return _out_x;
}

void CModelLegged::state2dh(const Eigen::VectorXf& _x, CDhRobotModel& _out_dh_model)
{
    if (_x.size() != ModelLegged_NumStates)
    {
        spdlog::error("ERROR CLeggedModel: state size different from robot model. Could not convert to DH");
        return;
    }

    for (CyC_UINT i = 0; i < _out_dh_model.m_Arms.size(); ++i)
    {
        std::vector<CLinkDH>* arm = _out_dh_model.m_Arms[i].getArmDH();
        for (CyC_UINT j = 0; j < arm->size(); ++j)
        {
            const CyC_UINT idx = i * (CyC_UINT)arm->size() + j;
            arm->at(j).theta = _x[idx * ModelLegged_JointStates + 0]; // Joint angle
            arm->at(j).theta_dot = _x[idx * ModelLegged_JointStates + 1]; // Joint angular velocity
            arm->at(j).theta_ddot = _x[idx * ModelLegged_JointStates + 2]; // Joint angular acceleration
        }
    }

    const size_t translationIndex = ModelLegged_NumJointStates;
    const size_t rotationIndex = translationIndex + ModelLegged_BodyPositionStates;

    const Eigen::Vector3f trans = _x.segment<ModelLegged_BodyPositionStates>(translationIndex);
    const Eigen::Vector4f rot = _x.segment<ModelLegged_BodyAttitudeStates>(rotationIndex);

    CPose body_pose(trans, rot);
    _out_dh_model.setBodyPose(body_pose);
}

void CModelLegged::state2dh(CDhRobotModel& _out_dh_model) const
{
    state2dh(m_x, _out_dh_model);
}

// Attempt to determine whether the entire body is moving or not
// based on the current joint angles and the control vector
bool CModelLegged::isMovingBody(const Eigen::VectorXf& u) const
{
    Eigen::MatrixXf currentAngles = Eigen::MatrixXf::Zero(ModelLegged_NumLegs, ModelLegged_NumJoints);
    Eigen::MatrixXf inputAngles = Eigen::MatrixXf::Zero(ModelLegged_NumLegs, ModelLegged_NumJoints);
    for (CyC_UINT i = 0; i < m_pDhRobotModel->m_Arms.size(); ++i)
    {
        const std::vector<CLinkDH>* arm = m_pDhRobotModel->m_Arms[i].getArmDH();
        for (CyC_UINT j = 0; j < arm->size(); ++j)
        {
            const CyC_UINT idx = i * (CyC_UINT)arm->size() + j;

            currentAngles(i, j) = arm->at(j).theta;
            inputAngles(i, j) = u(idx);
        }
    }

    Eigen::VectorXf norms = (currentAngles - inputAngles).rowwise().norm();
    //const bool anyZero = (norms.array() < 1e-6).any();
    const bool allNonZero = (norms.array() > 1e-3).all();
  
    return allNonZero;
}

Eigen::VectorXf CModelLegged::ikine(const Eigen::Vector3f& _desired_foot_pos_fr,
    const Eigen::Vector3f& _desired_foot_pos_fl,
    const Eigen::Vector3f& _desired_foot_pos_rr,
    const Eigen::Vector3f& _desired_foot_pos_rl,
    const CPose& _body_pose) const
{
    Eigen::VectorXf u = Eigen::VectorXf::Zero(ModelLegged_NumInputs);

    // TODO: remove hardcoded 0 0 body pose
    const Eigen::Vector3f q_fr = ikine_f(FOOT_INDEX::FOOT_FR, _desired_foot_pos_fr, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    const Eigen::Vector3f q_fl = ikine_f(FOOT_INDEX::FOOT_FL, _desired_foot_pos_fl, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    const Eigen::Vector3f q_rr = ikine_f(FOOT_INDEX::FOOT_RR, _desired_foot_pos_rr, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    const Eigen::Vector3f q_rl = ikine_f(FOOT_INDEX::FOOT_RL, _desired_foot_pos_rl, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

    u[0] = q_fl[0];
    u[1] = q_fl[1];
    u[2] = q_fl[2];

    u[3] = q_fr[0];
    u[4] = q_fr[1];
    u[5] = q_fr[2];

    u[6] = q_rl[0];
    u[7] = q_rl[1];
    u[8] = q_rl[2];

    u[9] = q_rr[0];
    u[10] = q_rr[1];
    u[11] = q_rr[2];

    return u;
}

Eigen::Vector3f CModelLegged::ikine_f(FOOT_INDEX idx, const Eigen::Vector3f& pstar, float tx, float ty, float tz, float Rx, float Ry, float Rz) const
{
    Eigen::Matrix4f DH = CPose::Rt2T(0, PI / 2, 0.f, 0.f, 0.f, 0.f);
    Eigen::Matrix4f Fr = m_pDhRobotModel->m_Arms[(size_t)idx].m_Arm2BasePose.transform();
    Eigen::Matrix4f Corp = CPose::Rt2T(Rz, Ry, Rx, -tz, ty, tx);

    Eigen::Matrix4f Base_fr = DH * Corp * Fr;

    const float x0_ = Base_fr(0, 3);
    const float y0_ = Base_fr(1, 3);
    const float z0_ = Base_fr(2, 3);

    float x_ = pstar(0);
    float y_ = pstar(1);
    float z_ = pstar(2);

    const Eigen::Matrix4f Origin = CPose::Rt2T(0.f, 0.f, 0.f, x0_, y0_, z0_);
    const Eigen::Matrix4f Point = CPose::Rt2T(0.f, 0.f, 0.f, x_ - x0_, y_ - y0_, z_ - z0_);
    const Eigen::Matrix4f Rot = CPose::Rt2T(-Rx, -Ry, Rz, 0.f, 0.f, 0.f);

    const Eigen::Matrix4f mat_pstar = Origin + Rot * Point;

    x_ = mat_pstar(0, 3);
    y_ = mat_pstar(1, 3);
    z_ = mat_pstar(2, 3);

    const float off = fabsf(m_pDhRobotModel->m_Arms[(size_t)idx].m_DHArmModel[1].d);
    const float L = m_pDhRobotModel->m_Arms[(size_t)idx].m_DHArmModel[1].a;

    const float m = sqrt((z_ - z0_) * (z_ - z0_));
    const float n = sqrt((y_ - y0_) * (y_ - y0_));
    const float p = sqrt((y0_ - y_) * (y0_ - y_) + (z0_ - z_) * (z0_ - z_));
    const float alp0 = atan(n / m);
    const float alp1 = asin(off / p);

    const float side_sign = ((idx == FOOT_INDEX::FOOT_FR) || (idx == FOOT_INDEX::FOOT_RR)) ? -1.f : 1.f;
    const float q1_ = side_sign * (alp0 - alp1);

    const float x = sqrt((x_ - x0_) * (x_ - x0_));
    const float r = sqrt(p * p - off * off);
    const float a = sqrt(r * r + x * x);

    const float fi = asin((x_ - x0_) / a);

    const float alpha = acos((a * a + L * L - L * L) / (2 * a * L));
    const float q2_ = alpha - fi;
    const float q3_ = -PI + acos((L * L + L * L - a * a) / (2 * L * L));

    return { q1_, q2_, q3_ };
}

ModelLeggedInput::ModelLeggedInput(const CycControlInput& input)
{
    fromControlInput(input);
}

void ModelLeggedInput::setControlType(size_t type)
{
    u[(size_t)Index::CTRL_TYPE] = static_cast<float>(type);
}

size_t ModelLeggedInput::getControlType() const
{
    return static_cast<size_t>(round(u[(size_t)Index::CTRL_TYPE]));
}

void ModelLeggedInput::setControllerState(size_t state)
{
    u[(size_t)Index::CTRL_STATE] = static_cast<float>(state);
}

size_t ModelLeggedInput::getControllerState() const
{
    return static_cast<size_t>(round(u[(size_t)Index::CTRL_STATE]));
}

bool ModelLeggedInput::setDesiredPosition(const Eigen::VectorXf& pos)
{
    if (pos.size() != ModelLegged_NumInputs)
    {
        spdlog::error("ModelLeggedInput::setDesiredPosition: Wrong vector size: {}", pos.size());
        return false;
    }

    u.segment((size_t)Index::POSITION, ModelLegged_NumInputs) = pos;
    return true;
}

Eigen::Ref<const Eigen::VectorXf> ModelLeggedInput::getDesiredPosition() const
{
    return u.segment((size_t)Index::POSITION, ModelLegged_NumInputs);
}

bool ModelLeggedInput::setDesiredVelocity(const Eigen::VectorXf& vel)
{
    if (vel.size() != ModelLegged_NumInputs)
    {
        spdlog::error("ModelLeggedInput::setDesiredVelocity: Wrong vector size: {}", vel.size());
        return false;
    }

    u.segment((size_t)Index::VELOCITY, ModelLegged_NumInputs) = vel;
    return true;
}

Eigen::Ref<const Eigen::VectorXf> ModelLeggedInput::getDesiredVelocity() const
{
    return u.segment((size_t)Index::VELOCITY, ModelLegged_NumInputs);
}

bool ModelLeggedInput::setDesiredAcceleration(const Eigen::VectorXf& acc)
{
    if (acc.size() != ModelLegged_NumInputs)
    {
        spdlog::error("ModelLeggedInput::setDesiredAcceleration: Wrong vector size: {}", acc.size());
        return false;
    }

    u.segment((size_t)Index::ACCEL, ModelLegged_NumInputs) = acc;
    return true;
}

Eigen::Ref<const Eigen::VectorXf> ModelLeggedInput::getDesiredAcceleration() const
{
    return u.segment((size_t)Index::ACCEL, ModelLegged_NumInputs);
}

bool ModelLeggedInput::setDesiredTorque(const Eigen::VectorXf& tau)
{
    if (tau.size() != ModelLegged_NumInputs)
    {
        spdlog::error("ModelLeggedInput::setDesiredTorque: Wrong vector size: {}", tau.size());
        return false;
    }

    u.segment((size_t)Index::TORQUE, ModelLegged_NumInputs) = tau;
    return true;
}

Eigen::Ref<const Eigen::VectorXf> ModelLeggedInput::getDesiredTorque() const
{
    return u.segment((size_t)Index::TORQUE, ModelLegged_NumInputs);
}

CycControlInput ModelLeggedInput::toControlInput() const
{
    CycControlInput input;
    input.u = u;

    return input;
}

bool ModelLeggedInput::fromControlInput(const CycControlInput& input)
{
    if (input.u.size() != (size_t)Index::SIZE)
    {
        spdlog::error("ModelLeggedInput::fromControlInput: Wrong vector size: {}", input.u.size());
        return false;
    }

    u = input.u;
    return true;
}