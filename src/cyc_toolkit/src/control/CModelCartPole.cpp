#include "CModelCartPole.h"

const CyC_INT CModelCartPole::m_NumStates = 4;
const CyC_INT CModelCartPole::m_NumInputs = 4;
const CyC_INT CModelCartPole::m_NumOutputs = 4;

CModelCartPole::CModelCartPole(const float _M, const float _m, const float _b, const float _l, const float _I, const float _g) : 
    CBaseStateSpaceModel(m_NumStates, m_NumInputs, m_NumOutputs)
{
    float c = (_I * (_M + _m) + _M * _m * _l * _l);

    // Define the system matrix
    Eigen::MatrixXf A = Eigen::MatrixXf(m_NumStates, m_NumStates);
    A << 0.f, 1.f, 0.f, 0.f,
        0.f, -(_I + _m * _l * _l) * _b / c, _m * _m * _g * _l * _l / c, 0.f,
        0.f, 0.f, 0.f, 1.f,
        0.f, -_m * _l * _b / c, _m * _g * _l * (_M + _m) / c, 0.f;

    // Define the input matrix
    Eigen::MatrixXf B = Eigen::MatrixXf(m_NumStates, m_NumInputs);
    B << 0.f,
        (_I + _m * _l * _l) / c,
        0.f,
        (_m * _l) / c;

    // Define the output matrix
    Eigen::MatrixXf C = Eigen::MatrixXf(m_NumOutputs, m_NumStates);
    C << 1.f, 0.f, 0.f, 0.f,
        0.f, 0.f, 1.f, 0.f;

    // Define the direct transmission matrix
    Eigen::MatrixXf D = Eigen::MatrixXf::Zero(m_NumOutputs, m_NumInputs);

    m_x = Eigen::VectorXf::Zero(m_NumStates);
    this->init(A, B, C, D);
}
