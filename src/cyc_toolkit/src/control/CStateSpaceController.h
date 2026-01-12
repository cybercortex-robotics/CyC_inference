// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CStateSpaceController_H_
#define CStateSpaceController_H_

#include "CyC_TYPES.h"
#include "CBaseController.h"

class CStateSpaceController : public CBaseController
{
public:
    CStateSpaceController(const CBaseStateSpaceModel& _state_space_model,
        const float _dt,
        const bool _enable_estimation = false,
        const bool _enable_integral_control = false,
        const bool _enable_reference_tracking = true,
        const bool _enable_lqr_approximation = false);
    CStateSpaceController(const CStateSpaceController&) = default;
    CStateSpaceController(CStateSpaceController&&) = default;
    CStateSpaceController& operator=(const CStateSpaceController&) = default;
    CStateSpaceController& operator=(CStateSpaceController&&) = default;
    ~CStateSpaceController() = default;

    // Setters
    bool setGains(const Eigen::MatrixXf& _K, const Eigen::MatrixXf& _L);

    virtual bool update(const Eigen::VectorXf& _y);

private:
    void computeK();
    void computeL();

    /*
        * Updates the control K and estimation L gains using LQR approximation
        *
        * Solve the discrete time LQR control problem
        * x[k + 1] = A x[k] + B u[k]
        * cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        */
    void dlqr_update();
    
private:
    bool m_EnableEstimation = false;
    bool m_EnableIntegralControl = false;
    bool m_EnableReferenceTracking = true;
    bool m_EnableLQRApproximation = false;

    // Control variables
    Eigen::VectorXf m_x_hat;    // State estimate
    Eigen::VectorXf m_w_hat;    // Estimate of a disturbance / error in the system model (used by the integral controller)

    // Control Gains
    Eigen::MatrixXf m_K;    // Controller Gain
    Eigen::MatrixXf m_L;    // Estimator Gain (L is equivilent to the Kalman Gain of a Kalman Filter.)
    Eigen::MatrixXf m_I;    // Integral control Gain

    // Calculates the expression A - L * C if state estimation is enabled
    Eigen::MatrixXf m_ALC;

    // N_bar maps the reference input r into an offset to the control input u (if reference tracking is enabled)
    Eigen::MatrixXf m_N_bar;
};

#endif /* CStateSpaceController_H_ */
