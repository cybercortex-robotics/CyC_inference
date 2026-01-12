#include "CBaseController.h"

CBaseController::CBaseController(const CBaseStateSpaceModel& _state_space_model, const float _dt) :
    m_StateSpaceModel(_state_space_model),
    m_dt(_dt)
{
    m_r = Eigen::VectorXf::Zero(m_StateSpaceModel.getNumOutputs());
    m_u = Eigen::VectorXf::Zero(m_StateSpaceModel.getNumInputs());
}

bool CBaseController::set_r(const Eigen::VectorXf _r)
{
    if (_r.size() != m_StateSpaceModel.getNumOutputs())
    {
        spdlog::error("{}: Size missmatch between given and controller reference. Cannot initialize reference vector.", typeid(*this).name());
        return false;
    }

    m_r = _r;

    return true;
}
