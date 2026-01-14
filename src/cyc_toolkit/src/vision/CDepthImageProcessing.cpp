// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CDepthImageProcessing.h"
#include <os/CCsvReader.h>

CDepthImageProcessing::CDepthImageProcessing()
{}

CycVoxel CDepthImageProcessing::depth2voxel(const CPinholeCameraSensorModel* _pSensorModel,
    const CycPoint& _pt,
    const float& _depth,
    const float& _scale)
{
    CycVoxel vx;
    vx.id = _pt.id;

    vx.pt3d.z() = _depth / _scale;
    vx.pt3d.x() = (_pt.pt2d.x() - _pSensorModel->cx()) * vx.pt3d.z() / _pSensorModel->fx_px();
    vx.pt3d.y() = (_pt.pt2d.y() - _pSensorModel->cy()) * vx.pt3d.z() / _pSensorModel->fy_px();
    vx.pt3d.w() = 1.f;

    return vx;
}

bool CDepthImageProcessing::depth2voxels(const CPinholeCameraSensorModel* _pSensorModel,
    const cv::Mat& _img_depth,
    CycVoxels& _out_voxels,
    const float& _scale,
    const CyC_INT _step,
    const cv::Mat& _img_rgb,
    std::vector<Eigen::Vector3i>& _out_colors)
{
    if (_img_depth.empty())
        return false;

    _out_voxels.clear();
    _out_colors.clear();

    CyC_INT channels = _img_depth.channels();
    for (CyC_INT i = 0; i < _img_depth.rows; i = i + _step)
    {
        uint8_t* rowPtr_depth = _img_depth.row(i).data;
        uint8_t* rowPtr_rgb;

        if (!_img_rgb.empty()) 
            rowPtr_rgb = _img_rgb.row(i).data;

        for (CyC_INT j = 0; j < _img_depth.cols; j = j + _step)
        {
            CycPoint pt((float)j, (float)i);
            uint8_t depth = rowPtr_depth[j];
            CycVoxel vx = depth2voxel(_pSensorModel, pt, (float)depth, _scale);
            _out_voxels.emplace_back(vx);

            if (!_img_rgb.empty())
            {
                Eigen::Vector3i color;
                color[2] = rowPtr_rgb[j * channels + 0]; // B
                color[1] = rowPtr_rgb[j * channels + 1]; // G
                color[0] = rowPtr_rgb[j * channels + 2]; // R
                _out_colors.emplace_back(color);
            }
        }
    }

    return true;
}

bool CDepthImageProcessing::keypoints2voxels(const CPinholeCameraSensorModel* _pSensorModel,
    const cv::Mat& _img_depth,
    const CycPoints& _pts,
    const float& _scale,
    CycVoxels& _out_voxels,
    CycPoints& _out_inliers)
{
    if (_img_depth.empty())
        return false;

    _out_voxels.clear();
    _out_inliers.clear();

    for (const auto& pt : _pts)
    {
        unsigned char dx;
        if (CImageProcessing::getPixelValue(_img_depth, static_cast<CyC_INT>(pt.pt2d.x()), static_cast<CyC_INT>(pt.pt2d.y()), dx))
        {
            float fDepth = static_cast<float>(dx);

            if (fDepth > 0.f)
            {
                _out_voxels.emplace_back(
                    depth2voxel(_pSensorModel, pt, fDepth, _scale)
                );

                _out_inliers.emplace_back(pt);
            }
        }
    }

    return true;
}

cv::Mat CDepthImageProcessing::depth2image(const cv::Mat& in_depth)
{
    // 3 channels of uint8 = 24 bits
    // 2^24 = 16777216
    // assuming max distance of 167.77216m => scale of 100000
    const float scale = 100000.F;

    // depth matrix has to be floating point with 1 channel
    assert(in_depth.type() == CV_32FC1);

    cv::Mat depth;
    in_depth.convertTo(depth, CV_32SC1, scale);

    cv::Mat image = cv::Mat::zeros(depth.size.operator()(), CV_8UC3);
    for (int i = 0; i < image.rows; ++i)
    {
        for (int j = 0; j < image.cols; ++j)
        {
            auto& px = image.at<cv::Vec3b>(i, j);
            auto distance = depth.at<int32_t>(i, j);
            for (int k = 0; k < image.channels(); ++k)
            {
                px[k] = (distance >> (8 * k)) & 0xFF;
            }
        }
    }

    return image;
}

cv::Mat CDepthImageProcessing::image2depth(const cv::Mat& in_image)
{
    // see CDepthImageProcessing::depth2image
    const float scale = 100000.F;

    assert(in_image.type() == CV_8UC3);

    cv::Mat depth = cv::Mat::zeros(in_image.size.operator()(), CV_32SC1);
    for (int i = 0; i < depth.rows; ++i)
    {
        for (int j = 0; j < depth.cols; ++j)
        {
            const auto& px = in_image.at<cv::Vec3b>(i, j);
            auto& distance = depth.at<int32_t>(i, j);
            for (int k = 0; k < in_image.channels(); ++k)
            {
                distance |= (static_cast<int32_t>(px[k]) << (8 * k));
            }
        }
    }

    cv::Mat out_depth;
    depth.convertTo(out_depth, CV_32FC1, 1.F / scale);

    return out_depth;
}

cv::Mat CDepthImageProcessing::asus2depth(const cv::Mat& in_depth)
{
    cv::Mat img_depth_meter = cv::Mat(in_depth.size(), CV_32FC1);
    for (CyC_INT y = 0; y < in_depth.rows; ++y)
        for (CyC_INT x = 0; x < in_depth.cols; ++x)
            img_depth_meter.at<float>(y, x) = (float)in_depth.at<uint16_t>(y, x) / 1000.f;
    return img_depth_meter;
}

bool CDepthImageProcessing::readVoxelsFile(const std::string& _voxels_path, CycVoxels& _out_voxels)
{
    _out_voxels.clear();

    if (!CFileUtils::FileExist(_voxels_path.c_str())) // needed for HDD images ?
    {
        spdlog::error("CDepthImageProcessing::readVoxelsFile(): ERROR Voxels file does not exist.");
    }
    else
    {
        csv::reader csv_reader;
        if (!csv_reader.open(_voxels_path))
        {
            spdlog::error("CDepthImageProcessing::readVoxelsFile(): failed to open csv '{}'", _voxels_path);
            return false;
        }

        csv_reader.select_cols("x", "y", "z", "id");
        float x, y, z;
        CyC_INT id;

        while (csv_reader.read_row(x, y, z, id))
            _out_voxels.emplace_back(CycVoxel{ Eigen::Vector4f{ x, y, z, 1.f }, id });
    }

    return true;
}
