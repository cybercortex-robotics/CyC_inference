// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Mihai Zaha

#ifndef CKALMAN_OPS_H
#define CKALMAN_OPS_H

#include <Eigen/Core>

namespace CKalmanOps
{
    void transitionState(
        Eigen::MatrixXf& x, // state
        Eigen::MatrixXf& P, // state covariance matrix
        const Eigen::MatrixXf& F, // state transition matrix
        const Eigen::MatrixXf& Q); // process noice covariance matrix

    void updateStateWithoutFilter(
        Eigen::MatrixXf& x,
        Eigen::MatrixXf& P,
        const Eigen::MatrixXf& z,
        const Eigen::MatrixXf& H,
        const Eigen::MatrixXf& R);

    bool updateStateWithFilter(
        Eigen::MatrixXf& x,
        Eigen::MatrixXf& P,
        const Eigen::MatrixXf& z,
        const Eigen::MatrixXf& H,
        const Eigen::MatrixXf& R);

    template <typename DecompositionType_t>
    float chiSquaredDistance(
        const DecompositionType_t& cho_factor,
        const Eigen::MatrixXf& y)
    {
        const Eigen::MatrixXf A = cho_factor.solve(y);
        const Eigen::VectorXf chi_distance = A.array().square().colwise().sum();
        return chi_distance(0);
    }

    float chiSquaredThreshold(
        const Eigen::MatrixXf& y);
}

#endif // CKALMAN_OPS_H
