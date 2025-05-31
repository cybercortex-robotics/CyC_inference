// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSTATESPACESIMULATION_H_
#define CSTATESPACESIMULATION_H_

#include "CyC_TYPES.h"
#include "CStateSpaceModel.h"


template<CyC_INT X, CyC_INT U, CyC_INT Y = X> class Simulation
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
