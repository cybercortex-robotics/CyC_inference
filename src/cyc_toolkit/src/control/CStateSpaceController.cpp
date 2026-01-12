#include "CStateSpaceController.h"

CStateSpaceController::CStateSpaceController(const CBaseStateSpaceModel& _state_space_model,
    const float _dt,
    const bool _enable_estimation,
    const bool _enable_integral_control,
    const bool _enable_reference_tracking,
    const bool _enable_lqr_approximation) :
    CBaseController(_state_space_model, _dt),
    m_EnableEstimation(_enable_estimation),
    m_EnableIntegralControl(_enable_integral_control),
    m_EnableReferenceTracking(_enable_reference_tracking),
    m_EnableLQRApproximation(_enable_lqr_approximation)
{
    const CyC_INT X = this->getModel()->getNumStates();
    const CyC_INT U = this->getModel()->getNumInputs();
    const CyC_INT Y = this->getModel()->getNumOutputs();

    // Control variables
    m_x_hat = Eigen::VectorXf(X);   // State estimate
    m_u     = Eigen::VectorXf(U);   // Control input
    m_r     = Eigen::VectorXf(Y);   // Reference input (assumed to be of the same dimension as the observation y)
    m_w_hat = Eigen::VectorXf(U);   // Estimate of a disturbance / error in the system model (used by the integral controller)

    // Control Gains
    m_K = Eigen::MatrixXf::Identity(U, X);  // Regulator Gain
    m_L = Eigen::MatrixXf::Identity(X, Y);  // Estimator Gain
    m_I = Eigen::MatrixXf::Identity(U, Y);  // Integral control Gain

    m_ALC   = Eigen::MatrixXf(X, X);
    m_N_bar = Eigen::MatrixXf(U, Y);
}

bool CStateSpaceController::setGains(const Eigen::MatrixXf& _K, const Eigen::MatrixXf& _L)
{
    if (_K.rows() != this->getModel()->getNumInputs() || _K.cols() != this->getModel()->getNumStates())
    {
        spdlog::error("{}: Size missmatch between given and controller gain. Cannot initialize controller gain.", typeid(*this).name());
        return false;
    }
    if (_L.rows() != this->getModel()->getNumStates() || _L.cols() != this->getModel()->getNumOutputs())
    {
        spdlog::error("{} Size of system matrix A differs from the given one.", typeid(*this).name());
        return false;
    }

    m_K = _K;
    m_L = _L;

    // TODO: Precalculate N_bar
    // ...



    m_ALC = this->getModel()->A() - m_L * this->getModel()->C();

    return true;
}

bool CStateSpaceController::update(const Eigen::VectorXf& _y)
{
    if (m_EnableLQRApproximation)
    {
        dlqr_update();
    }

    // If estimation is enabled, update the state estimate
    if (m_EnableEstimation)
    {
        m_x_hat += (m_ALC * m_x_hat + this->getModel()->B() * m_u + m_L * _y /*+ model.E*/) * this->dt();
    }
    // If not, then we assume that the entire state is fed back to the controller as y (i.e X == Y)
    else
    {
        const CyC_INT X = this->getModel()->getNumStates();
        const CyC_INT Y = this->getModel()->getNumOutputs();

        if (X != Y && !m_EnableEstimation)
        {
            spdlog::info("{}: Estimation must be enabled if the state is only partially observed (i.e len(X) != len(Y) )", typeid(*this).name());
            return false;
        }

        m_x_hat = _y;
    }

    // Calculate the control input required to drive the state to 0.
    m_u = -m_K * m_x_hat;

    // If reference tracking is enabled then offset the control input to drive the state to the reference input r
    // TBD
    if (m_EnableReferenceTracking)
    {
        m_u += m_N_bar * m_r;
    }

    // If integral control is enabled then windup the control input to offset a (presumably) constant disturbance w
    // TBD
    if (m_EnableIntegralControl)
    {
        m_w_hat += m_I * (_y - m_r) * this->dt();
        m_u += m_w_hat;
    }

    return true;
}

void CStateSpaceController::computeK()
{
    float eps = 0.01f; //1e-8;

    // Solve DARE (Discrete time Algebraic Riccati equation)
    Eigen::MatrixXf P_old = this->getModel()->Q();
    Eigen::MatrixXf P = this->getModel()->Q();

    for (int i = 0; i < 20000; ++i)
    {
        Eigen::MatrixXf invTerm = (this->getModel()->R() + this->getModel()->B().transpose() * P * this->getModel()->B());
        invTerm = invTerm.inverse();

        P = this->getModel()->Q() +
            (this->getModel()->A().transpose() * P * this->getModel()->A()) -
            (this->getModel()->A().transpose() * P * this->getModel()->B() * invTerm * this->getModel()->B().transpose() * P * this->getModel()->A());

        const Eigen::MatrixXf P_diff = (P - P_old);
        const Eigen::MatrixXf P_abs = P_diff.array().abs();

        if (P_abs.maxCoeff() < eps)
        {
            break;
        }

        P_old = P;

        ++i;
    }

    m_K = (this->getModel()->R() + this->getModel()->B().transpose() * P * this->getModel()->B()).inverse() * (this->getModel()->B().transpose() * P * this->getModel()->A());
}

void CStateSpaceController::computeL()
{
    static const Eigen::MatrixXf R = Eigen::MatrixXf::Identity(4, 4) * 0.01F;
    float eps = 0.01f; //1e-8;

    // Solve DARE (Discrete time Algebraic Riccati equation)
    Eigen::MatrixXf P_old = R;
    Eigen::MatrixXf P = R;

    for (int i = 0; i < 20000; ++i)
    {
        Eigen::MatrixXf invTerm = (this->getModel()->C() * P * this->getModel()->C().transpose() + R);
        invTerm = invTerm.inverse();

        P = this->getModel()->Q() +
            (this->getModel()->A() * P * this->getModel()->A().transpose()) -
            (this->getModel()->A() * P * this->getModel()->C().transpose() * invTerm * this->getModel()->C() * P * this->getModel()->A().transpose());

        const Eigen::MatrixXf P_diff = (P - P_old);
        const Eigen::MatrixXf P_abs = P_diff.array().abs();

        if (P_abs.maxCoeff() < eps)
        {
            break;
        }

        P_old = P;

        ++i;
    }

    m_L = (this->getModel()->A() * P * this->getModel()->C().transpose()) * (this->getModel()->C() * P * this->getModel()->C().transpose() + R).inverse();
}

void CStateSpaceController::dlqr_update()
{
    // Update the LQR approximation of the control gain matrix K
    computeK();

    // Update the reference tracking gain matrix
    if (m_EnableReferenceTracking)
    {
        Eigen::MatrixXf N = this->getModel()->A() - this->getModel()->B() * m_K;
        N = N.inverse();
        N = this->getModel()->C() * N * this->getModel()->B();

        // Case 1: more outputs than inputs - find the left inverse
        if (this->getModel()->getNumInputs() < this->getModel()->getNumOutputs())
        {
            m_N_bar = N.transpose() * N;
            m_N_bar = m_N_bar.inverse();
            m_N_bar = m_N_bar * N.transpose();
        }
        // Case 2: more than, or the same number of outputs as inputs - find the right inverse
        else
        {
            m_N_bar = N * N.transpose();
            m_N_bar = m_N_bar.inverse();
            m_N_bar = N.transpose() * m_N_bar;
        }

        if (Eigen::isnan(m_N_bar.array()).any())
        {
            m_N_bar.setZero();
        }
    }

    // Update the LQR approximation of the estimation gain matrix L
    if (m_EnableEstimation)
    {
        computeL();
        m_ALC = this->getModel()->A() - m_L * this->getModel()->C();
    }
}

