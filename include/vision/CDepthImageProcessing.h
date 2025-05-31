// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CDepthImageProcessing_H_
#define CDepthImageProcessing_H_

#include "CyC_TYPES.h"
#include "vision/CImageProcessing.h"
#include "sensors/CPinholeCameraSensorModel.h"

class CDepthImageProcessing
{
public:
    CDepthImageProcessing();
    CDepthImageProcessing(const CDepthImageProcessing&) = delete;
    CDepthImageProcessing(CDepthImageProcessing&&) = delete;
    CDepthImageProcessing operator=(const CDepthImageProcessing&) = delete;
    CDepthImageProcessing operator=(CDepthImageProcessing&&) = delete;
    ~CDepthImageProcessing() = default;

    /**
    * \brief Converts a 2D point having a depth to a voxel. Used with Kinect sensors.
    *
    * \param _pSensorModel      Pinhole camera sensor model
    * \param _pt                Image point
    * \param _depth             Depth value
    * \param _scale             Scale factor
    **/
    static CycVoxel depth2voxel(const CPinholeCameraSensorModel* _pSensorModel,
        const CycPoint& _pt,
        const float& _depth,
        const float& _scale);

    static bool depth2voxels(const CPinholeCameraSensorModel* _pSensorModel,
        const cv::Mat& _img_depth,
        CycVoxels& _out_voxels,
        const float& _scale,
        CyC_INT _step,
        const cv::Mat& _img_rgb,
        std::vector<Eigen::Vector3i>& _out_colors);

    static bool keypoints2voxels(const CPinholeCameraSensorModel* _pSensorModel, 
        const cv::Mat& _img_depth,
        const CycPoints& _pts,
        const float& _scale,
        CycVoxels& _out_voxels,
        CycPoints& _out_inliers);

    /**
    * \brief Encodes a depth image into a 3 channels RGB image.
    *
    * \param    in_depth Input depth image
    * \return   Encoded RGB image
    **/
    static cv::Mat depth2image(const cv::Mat& in_depth);
    
    /**
    * \brief Decodes a 3 channels RGB image into a depth image.
    *
    * \param    in_image Input RGB image
    * \return   Dencoded depth image
    **/
    static cv::Mat image2depth(const cv::Mat& in_image);

    /**
    * \brief Converts a raw ASUS depth image into meters.
    *
    * \param    in_depth Input depth image
    * \return   Depth image in meters
    **/
    static cv::Mat asus2depth(const cv::Mat& in_depth);

    static bool readVoxelsFile(const std::string& _voxels_path, CycVoxels& _out_voxels);
};

#endif /* CDepthImageProcessing_H_ */
