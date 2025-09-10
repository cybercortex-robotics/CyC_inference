// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CPlotFilterOutput_H_
#define CPlotFilterOutput_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include "CCycCore.h"
#include "vision/CImageDisplayUtils.h"

class CPlotFilterOutput
{
public:
    CPlotFilterOutput(CCycCore* vision_core);
    virtual ~CPlotFilterOutput();

    bool plotCycImage(cv::Mat& dst, CCycFilterBase* pFilter);
    bool plotCcrSemanticSegmentationImage(cv::Mat& dst, CCycFilterBase* pFilter);
    bool plotCycRoi2D(cv::Mat& dst, CCycFilterBase* pFilter);
    bool plotCycPoints(cv::Mat& dst, CCycFilterBase* pFilter);
    bool plotCycLanes(cv::Mat& dst, CCycFilterBase* pFilter);
    bool plotCycVoxels(cv::Mat& dst, CCycFilterBase* pFilter);
    bool plotCycPoses(cv::Mat& dst, CCycFilterBase* pFilter);
    bool plotCycBBox3D(cv::Mat& dst, CCycFilterBase* pFilter, CycDatablockKey keyOverlayFilter);
    bool plotUltrasonics(cv::Mat& dst, CCycFilterBase* pFilter);

private:
    CCycCore* m_pCycCore = nullptr;

    cv::Scalar m_TextColor = CV_RGB(20, 20, 255);
    CyC_INT m_nTextOffsetY = 17;

    std::unique_ptr<CObjectClasses> m_pObjectClasses;
};

#endif /* CPlotFilterOutput_H_ */