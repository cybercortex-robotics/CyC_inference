#include "CBaseStateSpaceModel.h"

CBaseStateSpaceModel::CBaseStateSpaceModel(const CyC_INT& _num_states, const CyC_INT& _num_inputs, const CyC_INT& _num_outputs) :
    m_nNumStates(_num_states),
    m_nNumInputs(_num_inputs),
    m_nNumOutputs(_num_outputs)
{
    const CyC_INT X = _num_states;
    const CyC_INT U = _num_inputs;
    const CyC_INT Y = _num_outputs;

    m_x = Eigen::VectorXf::Zero(X);
    m_y = Eigen::VectorXf::Zero(Y);

    m_A = Eigen::MatrixXf(X, X);
    m_B = Eigen::MatrixXf(X, U);
    m_C = Eigen::MatrixXf(Y, X);
    m_D = Eigen::MatrixXf(Y, U);

    // TBD - check if E should be removed   
    m_E = Eigen::MatrixXf(X, 1);

    // LQR cost function weight matrices
    m_Q = Eigen::MatrixXf(X, X);
    m_R = Eigen::MatrixXf(U, U);
}

bool CBaseStateSpaceModel::init(const Eigen::MatrixXf& _A,
    const Eigen::MatrixXf& _B,
    const Eigen::MatrixXf& _C,
    const Eigen::MatrixXf& _D)
{
    if (_A.rows() != m_nNumStates || _A.cols() != m_nNumStates)
    {
        spdlog::error("{} Size of system matrix A differs from the given one.", typeid(*this).name());
        return false;
    }
    if (_B.rows() != m_nNumStates || _B.cols() != m_nNumInputs)
    {
        spdlog::error("{} Size of input matrix B differs from the given one.", typeid(*this).name());
        return false;
    }
    if (_C.rows() != m_nNumOutputs || _C.cols() != m_nNumStates)
    {
        spdlog::error("{} Size of output matrix C differs from the given one.", typeid(*this).name());
        return false;
    }
    if (_D.rows() != m_nNumOutputs || _D.cols() != m_nNumInputs)
    {
        spdlog::error("{} Size of direct transmission matrix D differs from the given one.", typeid(*this).name());
        return false;
    }

    m_A = _A;
    m_B = _B;
    m_C = _C;
    m_D = _D;

    return true;
}

bool CBaseStateSpaceModel::step(const float& _dt, const Eigen::VectorXf& _u)
{
    m_x = m_x + (m_A * m_x + m_B * _u) * _dt;
    m_y = m_C * m_x;

    return true;
}

bool CBaseStateSpaceModel::set_x(const Eigen::VectorXf& _x)
{
    if (_x.size() != m_nNumStates)
    {
        spdlog::error("{}: Size missmatch between given and model state. Cannot initialize state vector.", typeid(*this).name());
        return false;
    }

    m_x = _x;

    return true;
}
bool CBaseStateSpaceModel::set_y(const Eigen::VectorXf& _y)
{
    if (_y.size() != m_nNumOutputs)
    {
        spdlog::error("{}: Size missmatch between given and model observations. Cannot initialize observations vector.", typeid(*this).name());
        return false;
    }

    m_y = _y;

    return true;
}

bool CBaseStateSpaceModel::str2state(const std::string& _str_state, CycState& _state)
{
    return str2state(_str_state, m_nNumStates, _state);
}

bool CBaseStateSpaceModel::str2state(const std::string& _str_state, const CyC_INT& _num_states, CycState& _state)
{
    std::vector<std::string> tokens;
    size_t pos = 0;
    std::string token;
    std::string delimiter = ",";
    CStringUtils::splitstring(_str_state, delimiter, tokens);

    if (tokens.size() != _num_states)
        return false;

    _state.x_hat.setConstant(_num_states, 0.0f);
    _state.x_hat(0) = std::stof(tokens[0]);
    _state.x_hat(1) = std::stof(tokens[1]);
    _state.x_hat(2) = std::stof(tokens[2]);
    _state.x_hat(3) = std::stof(tokens[3]);
    _state.x_hat(4) = std::stof(tokens[4]);

    return true;
}