// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Mihai Zaha

#include "CKalmanOps.h"

#include <Eigen/LU>
#include <Eigen/Cholesky>

namespace CKalmanOps
{
    void transitionState(
        Eigen::MatrixXf& x, // state
        Eigen::MatrixXf& P, // state covariance matrix
        const Eigen::MatrixXf& F, // state transition matrix
        const Eigen::MatrixXf& Q) // process noice covariance matrix
    {
        assert(x.rows() >= x.cols()); // column vector

        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void updateStateWithoutFilter(
        Eigen::MatrixXf& x,
        Eigen::MatrixXf& P,
        const Eigen::MatrixXf& z,
        const Eigen::MatrixXf& H,
        const Eigen::MatrixXf& R)
    {
        const Eigen::MatrixXf y = z - H * x;
        const Eigen::MatrixXf S = H * P * H.transpose() + R;

        const Eigen::MatrixXf cho_b = P * H.transpose();
        const auto cho_factor = S.ldlt();
        const Eigen::MatrixXf K = cho_factor.solve(cho_b.transpose()).transpose();

        x = x + K * y;
        P = P - K * S * K.transpose();
    }

    bool updateStateWithFilter(
        Eigen::MatrixXf& x,
        Eigen::MatrixXf& P,
        const Eigen::MatrixXf& z,
        const Eigen::MatrixXf& H,
        const Eigen::MatrixXf& R)
    {
        const Eigen::MatrixXf y = z - H * x;
        const Eigen::MatrixXf S = H * P * H.transpose() + R;

        const Eigen::MatrixXf cho_b = P * H.transpose();
        const auto cho_factor = S.ldlt();

        if (chiSquaredDistance(cho_factor, y) < chiSquaredThreshold(y))
        {
            const Eigen::MatrixXf K = cho_factor.solve(cho_b.transpose()).transpose();

            x = x + K * y;
            P = P - K * S * K.transpose();

            return true;
        }

        return false;
    }

    float chiSquaredThreshold(
        const Eigen::MatrixXf& y)
    {
        // Table for the 0.95 quantile of the chi-square distribution with N degrees of
        // freedom (contains values for N=1, ..., 9). Taken from MATLAB/Octave's chi2inv
        // function and used as Mahalanobis gating threshold.
        constexpr float chi2inv95[] = {
            0.F, // TODO
            3.8415F, // 1
            5.9915F, // 2...
            7.8147F,
            9.4877F,
            11.070F,
            12.592F,
            14.067F,
            15.507F,
            16.919F, // ...9
            18.307F,
            19.6751F,
            21.0261F,
            22.362F,
            23.6848F,
            24.9958F
        };

        constexpr size_t table_size = sizeof(chi2inv95) / sizeof(chi2inv95[0]);

        if (y.size() < table_size)
        {
            return chi2inv95[y.size()];
        }

        return -1.F;
    }
}


