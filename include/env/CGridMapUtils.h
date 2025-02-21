// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CGridMapUtils_H_
#define CGridMapUtils_H_

#include "CCR_TYPES.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include "vision/CImageDisplayUtils.h"

class CGridMapUtils
{
public:
    CGridMapUtils();
    virtual ~CGridMapUtils();

    /**
     * \brief Display a grid map in the A* coordinates system
     *
     * \param _disp_img   Output display image
     * \param _gridmap    Input gridmap
     * \param _angle      Vehicle yaw angle
     **/
    static void plotGridMap(cv::Mat& _disp_img, const Eigen::MatrixXi& _gridmap, const float& _octree_resolution, const float _angle = 0.F);
};

#endif /* CGridMapUtils_H_ */
