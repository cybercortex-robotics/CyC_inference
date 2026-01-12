// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CConcaveHull.h"
#include "concaveman.h"
#include <opencv2/imgproc.hpp>
#include <os/CTimer.h>

std::vector<Eigen::Vector4f> CConcaveHull::calculate(
    const std::vector<Eigen::Vector4f>& in_points)
{
    if (in_points.size() <= 2)
    {
        return in_points;
    }

    using T = float;
    using point_type = std::array<T, 2>;
    using vector_type = std::vector<point_type>;

    vector_type in_pts;
    in_pts.reserve(in_points.size());

    std::vector<cv::Point2f> in_cv_pts;
    in_cv_pts.reserve(in_points.size());

    for (const auto& pt : in_points)
    {
        in_pts.push_back({ pt.x(), pt.y() });
        in_cv_pts.emplace_back(pt.x(), pt.y());
    }

    std::vector<int> hull;
    cv::convexHull(in_cv_pts, hull);

    const vector_type out_pts = concaveman<float, 32>(in_pts, hull, 2.F, 0.F);

    std::vector<Eigen::Vector4f> out_points;
    for (const auto& pt : out_pts)
    {
        out_points.emplace_back(pt[0], pt[1], 0.F, 1.F);
    }

    return out_points;
}

std::vector<Eigen::Vector4f> CConcaveHull::calculate(
    const std::vector<Eigen::VectorXf>& in_points)
{
    std::vector<Eigen::Vector4f> cvt_in_points;
    cvt_in_points.reserve(in_points.size());

    for (size_t i = 0; i < in_points.size(); ++i)
    {
        cvt_in_points.emplace_back(in_points[i][0], in_points[i][1], in_points[i][2], in_points[i][3]);
    }

    const auto out_points = calculate(cvt_in_points);
    return out_points;
}
