// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CPolynomialFitting.h"

Eigen::VectorXf CPolynomialFitting::polyfit(const Eigen::VectorXf& xvals, const Eigen::VectorXf& yvals, CyC_INT order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXf A(xvals.size(), order + 1);

    for (CyC_INT i = 0; i < xvals.size(); ++i)
    {
        A(i, 0) = 1.0;
    }

    for (CyC_INT j = 0; j < xvals.size(); ++j)
    {
        for (CyC_INT i = 0; i < order; ++i)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);

    return result;
}

float CPolynomialFitting::polyeval(const Eigen::VectorXf& coeffs, float x)
{
    float result = 0.0;
    for (auto i = 0; i < coeffs.size(); ++i)
    {
        result += coeffs[i] * powf(x, (float)i);
    }
    return result;
}
