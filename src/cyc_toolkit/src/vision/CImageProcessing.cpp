// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CImageProcessing.h"
#include "vision/CProjectiveGeometry.h"
#include "vision/CTriangulation.h"
#include "env/CObjectClasses.h"

CImageProcessing::CImageProcessing()
{}

bool CImageProcessing::corner_subpix(const cv::Mat& _img, CycPoints& _pts)
{
    if (_pts.size() == 0 || _img.empty())
        return false;

    std::vector<cv::Point2f> cv_pts;

    for (size_t i = 0; i < _pts.size(); ++i)
        cv_pts.emplace_back(cv::Point2f{ _pts[i].pt2d.x(), _pts[i].pt2d.y() });

    // define the criteria to stop and refine the corners
    cv::TermCriteria  criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);
    cv::cornerSubPix(_img, cv_pts, cv::Size(5, 5), cv::Size(-1, -1), criteria);

    for (size_t i = 0; i < _pts.size(); ++i)
    {
        _pts[i].pt2d.x() = cv_pts[i].x;
        _pts[i].pt2d.y() = cv_pts[i].y;
    }

    return true;
}

bool CImageProcessing::getPixelValue(const cv::Mat& _img, const CyC_INT& _x, const CyC_INT& _y, unsigned char& _value)
{
    if (_x < 0 || _y < 0 || _x >= _img.cols || _y >= _img.rows || _img.channels() != 1)
        return false;

    uint8_t* pRow = _img.row(_y).data;
    _value = pRow[_x];
    
    return true;
}

bool CImageProcessing::getPixelValue(const cv::Mat& _img, const CyC_INT& _x, const CyC_INT& _y, Eigen::Vector3i& _value)
{
    if (_x < 0 || _y < 0 || _x >= _img.cols || _y >= _img.rows || _img.channels() != 3)
        return false;

    uint8_t* pRow = _img.row(_y).data;
    _value[2] = pRow[_x * _img.channels() + 0]; // B
    _value[1] = pRow[_x * _img.channels() + 1]; // G
    _value[0] = pRow[_x * _img.channels() + 2]; // R

    return true;
}

void CImageProcessing::simulateImg(const CPinholeCameraSensorModel* _pCamSensorModel,
    const CycVoxels& _voxels,
    cv::Mat& _out_img_rgb)
{
    cv::Mat _unused;
    simulateImg(_pCamSensorModel, _voxels, _out_img_rgb, _unused);
}

void CImageProcessing::simulateImg(const CPinholeCameraSensorModel* _pCamSensorModel,
    const CycVoxels& _voxels,
    cv::Mat& _out_img_rgb,
    cv::Mat& _out_img_depth)
{
    _out_img_rgb = cv::Mat(_pCamSensorModel->height(), _pCamSensorModel->width(), CV_8UC3, color::cyc_background);
    _out_img_depth = cv::Mat::zeros(cv::Size(_pCamSensorModel->width(), _pCamSensorModel->height()), CV_32F);

    writeCenteredText(_out_img_rgb, "CyberCortex Robotics");

    Pmatrix P = CProjectiveGeometry::KT2P(_pCamSensorModel->K(), CProjectiveGeometry::invertT(_pCamSensorModel->pose().transform()));

    // Filter for voxels in front of the camera
    CycVoxels pos_depth_voxels; CycPoints keypts;
    for (const CycVoxel& vx : _voxels)
    {
        float depth = CTriangulation::getDepth(vx, P);
        if (depth > 0.f)
        {
            CycPoint obs = CProjectiveGeometry::project(_pCamSensorModel, vx);
            
            depth = fabsf(_pCamSensorModel->pose().translation_3x1().z() - vx.pt3d.z());
            if (depth > 255.f)
                depth = 255.f;

            cv::circle(_out_img_depth,
                cv::Point2f(obs.pt2d.x(), obs.pt2d.y()),
                3,
                depth,
                -1);

            pos_depth_voxels.emplace_back(vx);
            keypts.emplace_back(obs);
        }
    }

    CyC_INT len = 3;
    for (const CycPoint& pt : keypts)
    {
        cv::line(_out_img_rgb, cv::Point2f{ pt.pt2d.x() - len, pt.pt2d.y() }, cv::Point2f{ pt.pt2d.x() + len, pt.pt2d.y() }, CV_RGB(0, 0, 0));
        cv::line(_out_img_rgb, cv::Point2f{ pt.pt2d.x(), pt.pt2d.y() - len }, cv::Point2f{ pt.pt2d.x(), pt.pt2d.y() + len }, CV_RGB(0, 0, 0));
    }
}

void CImageProcessing::writeCenteredText(cv::Mat& _out_img_rgb, const std::string& _str)
{
    int fontFace = cv::FONT_HERSHEY_DUPLEX;
    double fontScale = 1.5;
    int thickness = 3;
    int baseline = 0;

    cv::Size textSize = cv::getTextSize(_str, fontFace, fontScale, thickness, &baseline);

    // Formula: (ImageCenter) - (Half of TextSize)
    cv::Point textOrg((_out_img_rgb.cols - textSize.width) / 2,
        (_out_img_rgb.rows + textSize.height) / 2);

    cv::putText(_out_img_rgb,
        _str,
        textOrg,
        fontFace,
        fontScale,
        color::white,
        thickness);
}
