// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CyC_CONCAVEHULL_H_
#define CyC_CONCAVEHULL_H_

#include <CyC_TYPES.h>

class CConcaveHull
{
public:
    static std::vector<Eigen::Vector4f> calculate(
        const std::vector<Eigen::Vector4f>& in_points);

    static std::vector<Eigen::Vector4f> calculate(
        const std::vector<Eigen::VectorXf>& in_points);
};

#endif
