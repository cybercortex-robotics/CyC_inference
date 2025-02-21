// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSensorFusion_H_
#define CSensorFusion_H_

#include "CCR_TYPES.h"
#include <octomap/octomap.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include "sensors/CPinholeCameraSensorModel.h"
#include "env/CObjectClasses.h"

class CSensorFusion
{ 
public:
    CSensorFusion() {};
	virtual ~CSensorFusion() = default;

    /**
     * \brief TBD
     *
     * \param _input_depth_img      Input kinect depth image in int16_t format
     * \param _octree               Output 3D octree
     * \param _octree_depth_range   Input octree depth range
     **/
    bool accumulateOctree();
};

#endif /* CSensorFusion_H_ */
