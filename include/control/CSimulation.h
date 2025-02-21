// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSTATESPACESIMULATION_H_
#define CSTATESPACESIMULATION_H_

#include "CCR_TYPES.h"
#include "CStateSpaceModel.h"


template<CCR_INT X, CCR_INT U, CCR_INT Y = X> class Simulation
{
public:
    Eigen::VectorXf x = Eigen::VectorXf(X);

    const StateSpaceModel<X, U, Y>& model;

    Simulation(const StateSpaceModel<X, U, Y>& _model) :
        model(_model)
    {
        x = Eigen::VectorXf::Zero(X);
    }

    Eigen::VectorXf step(const Eigen::VectorXf& u, const float dt)
    {
        x += (model.A * x + model.B * u /*+ model.E*/) * dt;
        return model.C * x;
    }
};

#endif /* CSTATESPACESIMULATION_H_ */
