// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSTATESPACEMODEL_H_
#define CSTATESPACEMODEL_H_

#include "CyC_TYPES.h"
#include <iostream>
 
 /*
 * This defines a Model class to represent a state space model. Model has three template parameters:
 *
 * X := the number of states
 * U := the number of control inputs (inputs to the model, outputs from the controller)
 * Y := the number of outputs (sensor measurements of the system which are fed back to the controller)
 *
 * You can define any state space model by declaring a Model<X,U,Y> and filling out the system matrics, A, B, C & D appropriately.
 */
template<CyC_INT X, CyC_INT U, CyC_INT Y = X>
struct StateSpaceModel
{
    const static CyC_INT states = X;
    const static CyC_INT inputs = U;
    const static CyC_INT outputs = Y;

    Eigen::MatrixXf A = Eigen::MatrixXf(X, X);
    Eigen::MatrixXf B = Eigen::MatrixXf(X, U);
    Eigen::MatrixXf C = Eigen::MatrixXf(Y, X);
    Eigen::MatrixXf D = Eigen::MatrixXf(Y, U);

    // TBD - check if E should be removed
    Eigen::MatrixXf E = Eigen::MatrixXf(X, 1);

    // LQR cost function weight matrices
    Eigen::MatrixXf Q = Eigen::MatrixXf(X, X);  // The elements on the Q diagonal represent how important it is to tighly control the corresponding state element
    Eigen::MatrixXf R = Eigen::MatrixXf(U, U);  // The elements on the R diagonal represent how important it is to minimise the use of the corresponding control input
};

/*
 * This model describes the classic inverted pendulum control problem. In this system, a stick mounted on the top of a cart via passive revolute joint. The task is to keep this upright by
 * by accelerating the cart backwards and forwards. The actual modeling for this came from http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
 * which defines the state as:
 *
 * x := [cart_position, cart_velocity, stick_angle, stick_angular_rate]^T
 *
 * And it is assumed that 1: a force is directly applied to the cart and 2: that the system is equipped with sensors that can directly measure the cart's displacement and its stick angle.
 *
 * To parameterise the system the following physical measurements are required:
 *
 * M := mass of the cart                       (kg)
 * m := mass of the pendulum                   (kg)
 * b := coefficient of friction for cart       (N/m/s)
 * l := length to pendulum center of mass      (m)
 * I := mass moment of inertia of the pendulum (kg.m^2)
 */
struct CartPoleModel : public StateSpaceModel<4, 1, 2>
{
    CartPoleModel(float M, float m, float b, float l, float I, float g = 9.81f)
    {
        float c = (I * (M + m) + M * m * l * l);

        // Define the system matrix
        A << 0.f, 1.f, 0.f, 0.f,
            0.f, -(I + m * l * l) * b / c, m * m * g * l * l / c, 0.f,
            0.f, 0.f, 0.f, 1.f,
            0.f, -m * l * b / c, m * g * l * (M + m) / c, 0.f;

        // Define the input matrix
        B << 0.f,
            (I + m * l * l) / c,
            0.f,
            (m * l) / c;

        // Define the output matrix
        C << 1.f, 0.f, 0.f, 0.f,
             0.f, 0.f, 1.f, 0.f;

        // Define the direct transmission matrix
        D = Eigen::MatrixXf::Zero(2, 1);
    }
};

#endif /* CSTATESPACEMODEL_H_ */
