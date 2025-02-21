// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CImageProcessing_H_
#define CImageProcessing_H_

#include "CCR_TYPES.h"
#include <opencv2/imgproc.hpp>
#include "sensors/CPinholeCameraSensorModel.h"

class CImageProcessing
{
public:
    CImageProcessing();
    CImageProcessing(const CImageProcessing&) = delete;
    CImageProcessing(CImageProcessing&&) = delete;
    CImageProcessing operator=(const CImageProcessing&) = delete;
    CImageProcessing operator=(CImageProcessing&&) = delete;
    ~CImageProcessing() = default;

    static bool corner_subpix(const cv::Mat& _img, CcrPoints& _pts);
    
    static bool getPixelValue(const cv::Mat& _img, const CCR_INT& _x, const CCR_INT& _y, unsigned char& _value);
    static bool getPixelValue(const cv::Mat& _img, const CCR_INT& _x, const CCR_INT& _y, Eigen::Vector3i& _value);

    static void simulateImg(const CPinholeCameraSensorModel* _pCamSensorModel,
        const CcrVoxels& _voxels,
        cv::Mat& _out_img_rgb);

    static void simulateImg(const CPinholeCameraSensorModel* _pCamSensorModel,
        const CcrVoxels& _voxels,
        cv::Mat& _out_img_rgb, 
        cv::Mat& _out_img_depth);
};

#endif /* CImageProcessing_H_ */
