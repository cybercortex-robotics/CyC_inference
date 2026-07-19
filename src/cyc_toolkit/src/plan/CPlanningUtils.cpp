// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CPlanningUtils.h"
#include <iostream>

CPlanningUtils::CPlanningUtils()
{}

CPlanningUtils::~CPlanningUtils()
{}

void CPlanningUtils::plotReferenceSetpointsOnGridmap(cv::Mat& _disp_img, const CycSetPoints& _local_ref_path, const Eigen::MatrixXi& _gridmap, const float _angle, const std::vector<Eigen::Vector2f>& _fake__obstacles)
{
    cv::Size size = _disp_img.size();

    // Get the size of a gridmap cell in pixels
    cv::Size cellSize = cv::Size(round(size.width / _gridmap.cols()), round(size.height / _gridmap.rows()));
    
    CPose T_gridmap_rotation(float(_gridmap.rows() / 2), (float)(_gridmap.cols() / 2), 0.F, 0.F, 0.F, _angle + DEG2RAD * 90.F);

    // Draw the control reference path
    for (CyC_UINT i = 0; i < _local_ref_path.size(); ++i)
    {
        CyC_INT x_coord = _local_ref_path.at(i).r(0);
        CyC_INT y_coord = _local_ref_path.at(i).r(1);

        Eigen::Vector4f before_transform((float)x_coord - _gridmap.rows() / 2, (float)y_coord - _gridmap.cols() / 2, 0.F, 1.f);
        Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);
        after_transform = T_gridmap_rotation.transform() * before_transform;

        cv::Point pt1 = cv::Point((CyC_UINT)after_transform(0) * cellSize.width, (CyC_UINT)after_transform(1) * cellSize.height);
        cv::Point pt2 = cv::Point((CyC_UINT)after_transform(0) * cellSize.width + cellSize.width, (CyC_UINT)after_transform(1) * cellSize.height + cellSize.height);

        cv::rectangle(_disp_img, pt1, pt2, color::reference_path, cv::FILLED);
        //cv::rectangle(_disp_img, rect, cv::Scalar(0));
    }
}

void CPlanningUtils::plotPath(cv::Mat& _disp_img, const CycSetPoints& _global_mission_path, const CycState& _vehicle_state, const Eigen::MatrixXi& _gridmap, const float& _octree_resolution, const cv::Scalar& _color)
{
    // Rotation with PI radians (180 degrees) around the X axis is necessary because we need to flip the Y axis because:
    // in car coordinates Y axis increases to the left of the X axis
    // in image coordinates Y axis increases to the right of the Y axis
    
    CPose T_globalpath2vehicle(0.F, 0.F, 0.F, PI, 0.f, 0.f);

    const float scale_x = _disp_img.cols / (_gridmap.cols() * _octree_resolution);
    const float scale_y = _disp_img.rows / (_gridmap.rows() * _octree_resolution);

    const cv::Point2f ego_vehicle_origin{ _disp_img.rows / 2.f, _disp_img.cols / 2.f };
    
    CyC_INT nan = std::numeric_limits<CyC_INT>::quiet_NaN();
    cv::Point cvPtPrev(nan, nan);
    for (const auto& pt : _global_mission_path)
    {
        // Subtract the pivot point coordinates in order to rotate around the current position of the car
        Eigen::Vector4f before_transform((float)pt.r.x() - _vehicle_state.x_hat(0), (float)pt.r.y() - _vehicle_state.x_hat(1), 0.F, 1.f);
        Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);
        after_transform = T_globalpath2vehicle.transform() * before_transform;

        const float x = ego_vehicle_origin.x + roundf((after_transform(0)) * scale_x);
        const float y = ego_vehicle_origin.y + roundf((after_transform(1)) * scale_y);

        if (x > 0.f && y > 0.f && x < (float)_disp_img.cols && y < (float)_disp_img.rows)
        {
            cv::Point cvPt(x, y);
            if (cvPtPrev.x != nan && cvPtPrev.x > 0.f && cvPtPrev.y > 0.f)
                cv::line(_disp_img, cvPtPrev, cvPt, _color, 1);

            cvPtPrev = cvPt;
        }
    }
}

void CPlanningUtils::plotReferenceSetpoints(cv::Mat& _disp_img, const CycSetPoints& _ref_setpoints, const CycState& _vehicle_state, const Eigen::MatrixXi& _gridmap, const float& _octree_resolution)
{
    // Rotation with PI radians (180 degrees) around the X axis is necessary because we need to flip the Y axis because:
    // in car coordinates Y axis increases to the left of the X axis
    // in image coordinates Y axis increases to the right of the Y axis
    CPose T_globalpath2vehicle(0.F, 0.F, 0.F, PI, 0.f, 0.f);

    const auto scale_x = _disp_img.cols / (_gridmap.cols() * _octree_resolution);
    const auto scale_y = _disp_img.rows / (_gridmap.rows() * _octree_resolution);

    const cv::Point2f ego_vehicle_origin{ _disp_img.rows / 2.f, _disp_img.cols / 2.f };

    cv::Point ptPrev{ -1, -1 };
    for (const auto& pt : _ref_setpoints)
    {
        // Subtract the pivot point coordinates in order to rotate around the current position of the car
        Eigen::Vector4f before_transform(pt.r.x() - _ref_setpoints.front().r.x(), pt.r.y() - _ref_setpoints.front().r.y(), 0.F, 1.f);
        Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);
        after_transform = T_globalpath2vehicle.transform() * before_transform;

        const auto x = ego_vehicle_origin.x + roundf((after_transform(0)) * scale_x);
        const auto y = ego_vehicle_origin.y + roundf((after_transform(1)) * scale_y);
        cv::Point transformed_pt = cv::Point(x, y);

        if (x > 0 && y > 0 && x < _disp_img.cols && y < _disp_img.rows)
        {
            if (ptPrev.x > 0 && ptPrev.y > 0)
                cv::line(_disp_img, ptPrev, transformed_pt, color::reference_path, 2);
        }

        ptPrev = cv::Point(x, y);
    }
}

void CPlanningUtils::plotAStarGoalPoint(cv::Mat& _disp_img, const CycSetPoint& _goal_point, const CycState& _vehicle_state, const Eigen::MatrixXi& _gridmap, const float& _octree_resolution)
{
    CPose T_globalpath2vehicle(0.F, 0.F, 0.F, 180.F * DEG2RAD, 0.F, 0.F);

    const auto scale_x = _disp_img.cols / (_gridmap.cols() * _octree_resolution);
    const auto scale_y = _disp_img.rows / (_gridmap.rows() * _octree_resolution);

    const cv::Point2f ego_vehicle_origin{ _disp_img.rows / 2.f, _disp_img.cols / 2.f };

    Eigen::Vector4f before_transform(_goal_point.r(0) - _vehicle_state.x_hat(0), _goal_point.r(1) - _vehicle_state.x_hat(1), 0.F, 1.f);
    Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);
    after_transform = T_globalpath2vehicle.transform() * before_transform;

    const auto x = ego_vehicle_origin.x + roundf((after_transform(0)) * scale_x);
    const auto y = ego_vehicle_origin.y + roundf((after_transform(1)) * scale_y);

    if (x > 0 && y > 0 && x < _disp_img.cols && y < _disp_img.rows)
        cv::circle(_disp_img, cv::Point(x, y), 9, cv::Scalar(0, 255, 255), -1);
}

void CPlanningUtils::plotCandidateTrajectories(cv::Mat & _disp_img, const std::vector<CycSetPoints>& _candidate_trajectories, const CycState & _vehicle_state, const Eigen::MatrixXi & _gridmap, const float & _octree_resolution)
{
    // Rotation with PI radians (180 degrees) around the X axis is necessary because we need to flip the Y axis because:
    // in car coordinates Y axis increases to the left of the X axis
    // in image coordinates Y axis increases to the right of the Y axis
    CPose T_globalpath2vehicle(0.0F, 0.0F, 0.0F, PI, 0.f, 0);

    const auto scale_x = _disp_img.cols / (_gridmap.cols() * _octree_resolution);
    const auto scale_y = _disp_img.rows / (_gridmap.rows() * _octree_resolution);
    const cv::Point2f ego_vehicle_origin{ _disp_img.rows / 2.f, _disp_img.cols / 2.f };

    for (auto trajectory : _candidate_trajectories)
    {
        for (const auto& pt : trajectory)
        {
            // Subtract the pivot point coordinates in order to rotate around the current position of the car
            Eigen::Vector4f before_transform(
                pt.r.x() - trajectory.front().r.x(),
                pt.r.y() - trajectory.front().r.y(), 0.F, 1.f);

            Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);
            after_transform = T_globalpath2vehicle.transform() * before_transform;

            const auto x = ego_vehicle_origin.x + roundf((after_transform(0)) * scale_x);
            const auto y = ego_vehicle_origin.y + roundf((after_transform(1)) * scale_y);

            if (x > 0 && y > 0 && x < _disp_img.cols && y < _disp_img.rows)
                cv::circle(_disp_img, cv::Point(x, y), 1, cv::Scalar(232, 193, 39), -1);
        }
    }
}

size_t CPlanningUtils::findClosestPoint(const std::vector<CycSetPoint>& _mission_path,
    const CycState& _vehicle_state,
    size_t _previous_trajectory_point_index,
    size_t _closest_index)
{
    Eigen::MatrixXf mission_mat(2, _mission_path.size());
    for (size_t i = 0; i < _mission_path.size(); ++i)
    {
        mission_mat.col(i) << _mission_path[i].r.x(), _mission_path[i].r.y();
    }

    mission_mat.colwise() -= _vehicle_state.x_hat.topRows(2);
    const Eigen::VectorXf distances = mission_mat.colwise().squaredNorm();

    size_t selected_index = _previous_trajectory_point_index;
    float min_dist = std::numeric_limits<float>::max();
    auto end_index = _closest_index ? _closest_index : 150;
    auto stop_idx = std::min((size_t)distances.size(), _previous_trajectory_point_index + end_index);
    for (size_t idx = _previous_trajectory_point_index; idx < stop_idx; ++idx)
    {
        if (distances[idx] < min_dist)
        {
            min_dist = distances[idx];
            selected_index = idx;
        }
    }

    if ((selected_index == _previous_trajectory_point_index) &&
        ((selected_index + 1) < _mission_path.size()))
    {
        selected_index++;
    }

    return selected_index;
}
