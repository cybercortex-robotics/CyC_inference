// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CBaseController_H_
#define CBaseController_H_

#include "CyC_TYPES.h"
#include "CBaseStateSpaceModel.h"

class CBaseController
{
public:
    CBaseController(const CBaseStateSpaceModel& _state_space_model, const float _dt);
    CBaseController(const CBaseController&) = default;
    CBaseController(CBaseController&&) = default;
    CBaseController& operator=(const CBaseController&) = default;
    CBaseController& operator=(CBaseController&&) = default;
    ~CBaseController() = default;

    // Getter functions
    const float&                dt() const { return m_dt; }; 
    const Eigen::VectorXf&      r() const { return m_r; };
    const Eigen::VectorXf&      u() const { return m_u; };
    const CBaseStateSpaceModel* getModel() const { return &m_StateSpaceModel; };

    // Setter functions
    void    set_dt(const float _dt) { m_dt = _dt; };
    bool    set_r(const Eigen::VectorXf _r);

    // Calculates control output (must be overloaded by derived controller)
    virtual bool update(const Eigen::VectorXf& _y) = 0;

protected:
    Eigen::VectorXf m_r;    // Reference input (assumed to be of the same dimension as the observation y)
    Eigen::VectorXf m_u;    // Control input

private:
    // System model
    const CBaseStateSpaceModel& m_StateSpaceModel;

    float m_dt = 0.1f;    // Sampling time [s]
};

#endif /* CBaseController_H_ */
