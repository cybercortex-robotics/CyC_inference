// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/*
 *
 * This defines a Model class to represent a state space model. Model has three template parameters:
 *
 * X := the number of states
 * U := the number of control inputs (inputs to the model, outputs from the controller)
 * Y := the number of outputs (sensor measurements of the system which are fed back to the controller)
 *
 * You can define any state space model by declaring a CBaseStateSpaceModel(X,U,Y) and filling out the system matrics, A, B, C & D appropriately.
 *
 * https://github.com/tomstewart89/StateSpaceControl
 */

#ifndef CBaseStateSpaceModel_H_
#define CBaseStateSpaceModel_H_

#include <CCR_TYPES.h>

class CBaseStateSpaceModel
{
public:
    CBaseStateSpaceModel(const CCR_INT& _num_states, const CCR_INT& _num_inputs, const CCR_INT& _num_outputs);
    CBaseStateSpaceModel(const CBaseStateSpaceModel&) = default;
    CBaseStateSpaceModel(CBaseStateSpaceModel&&) = default;
    CBaseStateSpaceModel& operator=(const CBaseStateSpaceModel&) = default;
    CBaseStateSpaceModel& operator=(CBaseStateSpaceModel&&) = default;
    ~CBaseStateSpaceModel() = default;

    // Getter functions
    const bool isInitialized() const { return m_bInitialized; };
    const CCR_INT getNumStates() const { return m_nNumStates; };
    const CCR_INT getNumInputs() const { return m_nNumInputs; };
    const CCR_INT getNumOutputs() const { return m_nNumOutputs; };

    const Eigen::VectorXf& x() const { return m_x; };
    const Eigen::VectorXf& y() const { return m_y; };
    const Eigen::MatrixXf& A() const { return m_A; };
    const Eigen::MatrixXf& B() const { return m_B; };
    const Eigen::MatrixXf& C() const { return m_C; };
    const Eigen::MatrixXf& D() const { return m_D; };
    const Eigen::MatrixXf& Q() const { return m_Q; };
    const Eigen::MatrixXf& R() const { return m_R; };

    // Setter functions
    bool set_x(const Eigen::VectorXf& _x);
    bool set_y(const Eigen::VectorXf& _y);

    // Simulation function (can be reimplemented by derived class)
    // Returns a vector of observations
    virtual bool step(const float& _dt, const Eigen::VectorXf& _u);

    bool str2state(const std::string& _str_state, CcrState& _state);
    static bool str2state(const std::string& _str_state, const CCR_INT& _num_states, CcrState& _state);

protected:
    // Setup system matrices
    bool init(const Eigen::MatrixXf& _A,
                  const Eigen::MatrixXf& _B,
                  const Eigen::MatrixXf& _C,
                  const Eigen::MatrixXf& _D);

protected:
    Eigen::VectorXf m_x;    // State
    Eigen::VectorXf m_y;    // Observations

    bool    m_bInitialized = false;
    CCR_INT m_nNumStates = 0;
    CCR_INT m_nNumInputs = 0;
    CCR_INT m_nNumOutputs = 0;

    Eigen::MatrixXf m_A;    // System matrix
    Eigen::MatrixXf m_B;    // Input matrix
    Eigen::MatrixXf m_C;    // Output matrix
    Eigen::MatrixXf m_D;    // Direct transmission matrix

    // TBD - check if E should be removed
    Eigen::MatrixXf m_E;

    // LQR cost function weight matrices
    Eigen::MatrixXf m_Q;    // The elements on the Q diagonal represent how important it is to tighly control the corresponding state element
    Eigen::MatrixXf m_R;    // The elements on the R diagonal represent how important it is to minimise the use of the corresponding control input
};

#endif // CBaseStateSpaceModel_H_
