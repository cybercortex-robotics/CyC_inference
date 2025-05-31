// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

#ifndef CPolynomialFitting_H_
#define CPolynomialFitting_H_

#include "CyC_TYPES.h"

class CPolynomialFitting
{
public:
    CPolynomialFitting();
    virtual ~CPolynomialFitting();

    static Eigen::VectorXf polyfit(const Eigen::VectorXf& xvals, const Eigen::VectorXf& yvals, CyC_INT order);
    static float polyeval(const Eigen::VectorXf& coeffs, float x);
};

#endif /* CPolynomialFitting_H_ */