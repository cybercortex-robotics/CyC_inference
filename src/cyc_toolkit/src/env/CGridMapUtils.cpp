// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CGridMapUtils.h"
#include <iostream>

CGridMapUtils::CGridMapUtils()
{}

CGridMapUtils::~CGridMapUtils()
{}

void CGridMapUtils::plotGridMap(cv::Mat& _disp_img, const Eigen::MatrixXi& _gridmap, const float& _octree_resolution, const float _angle)
{
    cv::Size size = cv::Size(1200, 1200);
    //cv::Size size = _disp_img.size();
    //_disp_img = cv::Mat(size, CV_8UC3, color::free_space);
    cv::Mat cvGrid = cv::Mat(size, CV_8UC3, color::free_space);

    // Get the size of a gridmap cell in pixels
    cv::Size cellSize = cv::Size(round(size.width / _gridmap.cols()), round(size.height / _gridmap.rows()));
    
    CPose T_gridmap_rotation(float(_gridmap.rows() / 2), (float)(_gridmap.cols() / 2), 0.F, 0.F, 0.F, _angle + DEG2RAD * 90.F);

    // Parse the gridmap and rotate the occupied cells according to the angle of the vehicle
    for (CyC_UINT i = 0, nRows = _gridmap.rows(), nCols = _gridmap.cols(); i < nRows; ++i)
    {
        for (CyC_UINT j = 0; j < nCols; ++j)
        {
            if (_gridmap(j, i) != CObjectClasses::UNDEFINED)
            {
                Eigen::Vector4f before_transform((float)i - _gridmap.rows() / 2, (float)j - _gridmap.cols() / 2, 0.F, 1.f);
                Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);
                after_transform = T_gridmap_rotation.transform() * before_transform;

                cv::Point pt1 = cv::Point((CyC_UINT)after_transform(0) * cellSize.width, (CyC_UINT)after_transform(1) * cellSize.height);
                cv::Point pt2 = cv::Point((CyC_UINT)after_transform(0) * cellSize.width + cellSize.width, (CyC_UINT)after_transform(1) * cellSize.height + cellSize.height);
                
                const auto& color = CObjectClasses::getColor(_gridmap(j, i));
                cv::rectangle(cvGrid, pt1, pt2, color, cv::FILLED);
            }
        }
    }
    
    // Draw the grid's x axis
    for (CyC_UINT i = 0; i < _gridmap.cols(); i += 3)
    {
        cv::Point pt1 = cv::Point((CyC_UINT)i * cellSize.width, 0);
        cv::Point pt2 = cv::Point((CyC_UINT)i * cellSize.width, cvGrid.rows - 1);
        cv::line(cvGrid, pt1, pt2, CV_RGB(150, 150, 150));
    }

    // Draw the grid's y axis
    for (CyC_UINT i = 0; i < _gridmap.rows(); i += 3)
    {
        cv::Point pt1 = cv::Point(0, (CyC_UINT)i * cellSize.height);
        cv::Point pt2 = cv::Point(cvGrid.cols - 1, (CyC_UINT)i * cellSize.height);
        cv::line(cvGrid, pt1, pt2, CV_RGB(150, 150, 150));
    }
    
    // Draw coordinate system and resolution
    //cv::Point ptOrg = cv::Point(2 * cellSize.width, (_gridmap.rows() - 1) * cellSize.height);
    //cv::Point ptX = cv::Point(2 * cellSize.width, (_gridmap.rows() - 2) * cellSize.height);
    //cv::Point ptY = cv::Point(cellSize.width, (_gridmap.rows() - 1) * cellSize.height);
    cv::Point ptOrg = cv::Point(50, (_gridmap.rows() - 1) * cellSize.height - 50);
    cv::Point ptX = cv::Point(100, (_gridmap.rows() - 1) * cellSize.height - 50);
    cv::Point ptY = cv::Point(50, (_gridmap.rows() - 1) * cellSize.height - 100);

    // Draw axes convention
    cv::arrowedLine(cvGrid, ptOrg, ptX, color::x_axis, 2);
    cv::arrowedLine(cvGrid, ptOrg, ptY, color::y_axis, 2);
    cv::circle(cvGrid, ptOrg, 8, color::z_axis, -1);

    char str[128];
    snprintf(str, sizeof(str) - 1, "Resolution: %.1f m x %.1f m", _octree_resolution, _octree_resolution);
    cv::putText(cvGrid, str, cv::Point(cellSize.width, ((_gridmap.rows() - 3) * cellSize.height)), cv::FONT_HERSHEY_PLAIN, 2., color::black, 2);

    cv::resize(cvGrid, _disp_img, _disp_img.size());
}
