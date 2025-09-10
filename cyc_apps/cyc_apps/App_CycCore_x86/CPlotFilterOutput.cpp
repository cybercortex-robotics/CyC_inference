// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CPlotFilterOutput.h"
#include "sensors/CPinholeCameraSensorModel.h"


CPlotFilterOutput::CPlotFilterOutput(CCycCore* vision_core) :
    m_pCycCore(vision_core)
{
    m_pObjectClasses = std::make_unique<CObjectClasses>("");
}

CPlotFilterOutput::~CPlotFilterOutput()
{}

bool CPlotFilterOutput::plotCcrSemanticSegmentationImage(cv::Mat& dst, CCycFilterBase* pFilter)
{
    bool bImageRead = CCycCore::getCycImage2CvMat(pFilter, dst);
    if (dst.cols > 900u || dst.rows > 700u)
        cv::resize(dst, dst, cv::Size(dst.cols / 2, dst.rows / 2));
    
    CycImages img;
    cv::Mat ccr_img;
    dst.copyTo(ccr_img);
    
    if (dst.channels() < 3)
        cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
    
    img.push_back(CycImage_(ccr_img));
    if (bImageRead)
        CImageDisplayUtils::drawSemanticSegmentation(dst, img[0]);
    return bImageRead;

}

bool CPlotFilterOutput::plotCycImage(cv::Mat& dst, CCycFilterBase* pFilter)
{
    bool bImageRead = CCycCore::getCycImage2CvMat(pFilter, dst);

    if (dst.cols >= 640u || dst.rows >= 480u)
        cv::resize(dst, dst, cv::Size(dst.cols / 1.2f, dst.rows / 1.2f));

    CImageDisplayUtils::printTimestamp(dst, pFilter->getTimestampStop());

    return bImageRead;
}

bool CPlotFilterOutput::plotUltrasonics(cv::Mat& dst, CCycFilterBase* pFilter)
{
    bool bIsSourceFilter(false);
    CyC_TIME_UNIT readTimeImage(-1);

    CycUltrasonics ultrasonics;

    bool bDataRead = pFilter->getData(ultrasonics);
    const CyC_UINT num_us_front = std::stoi(pFilter->m_CustomParameters["num_us_front"]);
    const CyC_UINT num_us_back = std::stoi(pFilter->m_CustomParameters["num_us_back"]);

    if (bDataRead)
    {
        CycState state;
        state.x_hat.resize(4);
        state.x_hat << 0.f, 0.f, 0.f, 1.57f;
        dst = CImageDisplayUtils::ultrasonicsToImage(state, ultrasonics, num_us_front, num_us_back);
    }

    return bDataRead;
}

bool CPlotFilterOutput::plotCycRoi2D(cv::Mat& dst, CCycFilterBase* pFilter)
{
    bool bImageRead(false);
    bool bIsSourceFilter(false);

    // Get the image source filter
    CycDatablockKey keyOverlayFilter;
    for (const CycInputSource& src : pFilter->getInputSources())
    {
        if (src.pCycFilter->getOutputDataType() == CyC_IMAGE)
        {
            keyOverlayFilter = src.pCycFilter->getFilterKey();
        }
    }

    CycRois2D objs;
    bool bDataRead = pFilter->getData(objs);

    CCycFilterBase* pOverlayFilter;
    m_pCycCore->readFilter(keyOverlayFilter, pOverlayFilter);

    // Check if the overlay filter is the source for computing the 2D ROIs
    if (pOverlayFilter == pFilter->getInputSources()[0].pCycFilter)
        bIsSourceFilter = true;

    if (pOverlayFilter != nullptr)
        bImageRead = CCycCore::getCycImage2CvMat(pOverlayFilter, dst);

    if (bDataRead && bImageRead)
    {
        std::unordered_map<CyC_INT, std::string> obj_classes_map = m_pObjectClasses->getLandmarks();
        CImageDisplayUtils::drawObjects(dst, objs, obj_classes_map, keyOverlayFilter);

        if (dst.cols > 900u || dst.rows > 700u)
            cv::resize(dst, dst, cv::Size(dst.cols / 2, dst.rows / 2));

        CImageDisplayUtils::printTimestamp(dst, pFilter->getTimestampStop());

        if (bIsSourceFilter)
            cv::putText(dst, "Synchronized detections", cv::Point(5, m_nTextOffsetY * 2), cv::FONT_HERSHEY_PLAIN, 1, m_TextColor, 1);
        else
            cv::putText(dst, "Unsynchronized detections", cv::Point(5, m_nTextOffsetY * 2), cv::FONT_HERSHEY_PLAIN, 1, m_TextColor, 1);
    }

    return bDataRead && bImageRead;
}

bool CPlotFilterOutput::plotCycPoints(cv::Mat& dst, CCycFilterBase* pFilter)
{
    bool bImageRead(false);
    bool bIsSourceFilter(false);

    // Get the image source filter
    CycDatablockKey keyOverlayFilter;
    for (const CycInputSource& src : pFilter->getInputSources())
    {
        if (src.pCycFilter->getOutputDataType() == CyC_IMAGE)
        {
            keyOverlayFilter = src.pCycFilter->getFilterKey();
        }
    }

    CycPoints pts;
    bool bDataRead = pFilter->getData(pts);

    CCycFilterBase* pOverlayFilter;
    m_pCycCore->readFilter(keyOverlayFilter, pOverlayFilter);

    // Check if the overlay filter is the source for computing the 2D ROIs
    if (pOverlayFilter == pFilter->getInputSources()[0].pCycFilter)
        bIsSourceFilter = true;

    if (pOverlayFilter != nullptr)
        bImageRead = CCycCore::getCycImage2CvMat(pOverlayFilter, dst);

    if (bDataRead && bImageRead)
    {
        CImageDisplayUtils::drawPoints(dst, pts, color::green);

        if (dst.cols > 900u || dst.rows > 700u)
            cv::resize(dst, dst, cv::Size(dst.cols / 2, dst.rows / 2));

        CImageDisplayUtils::printTimestamp(dst, pFilter->getTimestampStop());

        if (bIsSourceFilter)
            cv::putText(dst, "Synchronized detections", cv::Point(5, m_nTextOffsetY * 2), cv::FONT_HERSHEY_PLAIN, 1, m_TextColor, 1);
        else
            cv::putText(dst, "Unsynchronized detections", cv::Point(5, m_nTextOffsetY * 2), cv::FONT_HERSHEY_PLAIN, 1, m_TextColor, 1);
    }

    return bDataRead && bImageRead;
}

bool CPlotFilterOutput::plotCycLanes(cv::Mat& dst, CCycFilterBase* pFilter)
{
    bool bImageRead(false);
    bool bIsSourceFilter(false);

    // Get the image source filter
    CycDatablockKey keyOverlayFilter;
    for (const CycInputSource& src : pFilter->getInputSources())
    {
        if (src.pCycFilter->getOutputDataType() == CyC_IMAGE)
            keyOverlayFilter = src.pCycFilter->getFilterKey();
    }

    CycLanesModel lanes_model;
    bool bDataRead = pFilter->getData(lanes_model);

    CCycFilterBase* pOverlayFilter;
    m_pCycCore->readFilter(keyOverlayFilter, pOverlayFilter);

    // Check if the overlay filter is the source for computing the lanes
    if (pOverlayFilter == pFilter->getInputSources()[0].pCycFilter)
        bIsSourceFilter = true;

    if (pOverlayFilter != nullptr)
        bImageRead = CCycCore::getCycImage2CvMat(pOverlayFilter, dst);

    if (bDataRead && bImageRead)
    {
        if (pFilter->getSensorModel() != nullptr)
        {
            const CPinholeCameraSensorModel* pSensorModel = static_cast<const CPinholeCameraSensorModel*>(pFilter->getSensorModel());
            CImageDisplayUtils::drawLanes(dst, lanes_model, *pSensorModel);
        }

        if (dst.cols > 900u || dst.rows > 700u)
            cv::resize(dst, dst, cv::Size(dst.cols / 2, dst.rows / 2));

        CImageDisplayUtils::printTimestamp(dst, pFilter->getTimestampStop());

        if (bIsSourceFilter)
            cv::putText(dst, "Synchronized detections", cv::Point(5, m_nTextOffsetY * 2), cv::FONT_HERSHEY_PLAIN, 1, m_TextColor, 1);
        else
            cv::putText(dst, "Unsynchronized detections", cv::Point(5, m_nTextOffsetY * 2), cv::FONT_HERSHEY_PLAIN, 1, m_TextColor, 1);
    }

    return bDataRead && bImageRead;
}

bool CPlotFilterOutput::plotCycVoxels(cv::Mat& dst, CCycFilterBase* pFilter)
{
    CycVoxels voxels;
    bool bDataRead = pFilter->getData(voxels);

    if (bDataRead)
    {
        dst = cv::Mat::zeros(800, 800, CV_8UC3);
        
        // Draw voxels
        cv::Vec2f center{ dst.rows / 2.f, dst.cols / 2.f };
        float scale = 60.f;
        for (auto const& vx : voxels)
        {
            cv::Vec2f pt{ vx.pt3d.x(), vx.pt3d.y() };
            const cv::Vec2f point = center + (pt * scale);
            cv::circle(dst, cv::Point{ (int)point[0], (int)point[1] }, 1, color::lidar, -1);
        }

        // Draw axes convention
        cv::Point2f ptOrg(center);
        cv::Point2f ptX = cv::Point2f(ptOrg.x + 70, ptOrg.y);
        cv::Point2f ptY = cv::Point2f(ptOrg.x, ptOrg.y - 70);
        cv::arrowedLine(dst, ptOrg, ptX, color::x_axis, 2);
        cv::arrowedLine(dst, ptOrg, ptY, color::y_axis, 2);
        cv::circle(dst, ptOrg, 5, color::z_axis, -1);

        char str[64];
        CImageDisplayUtils::printTimestamp(dst, pFilter->getTimestampStop(), color::blue);
        snprintf(str, sizeof(str) - 1, "No voxels: %zd", voxels.size());
        cv::putText(dst, str, cv::Point(5, m_nTextOffsetY * 3), cv::FONT_HERSHEY_PLAIN, 1.5, color::blue, 1);
    }
    
    return bDataRead;
}

bool CPlotFilterOutput::plotCycPoses(cv::Mat& dst, CCycFilterBase* pFilter)
{
    bool bImageRead(false);
    bool bIsSourceFilter(false);
    
    CycPoses poses;
    bool bDataRead = pFilter->getData(poses);

    // Get the image source filter
    CycDatablockKey keyOverlayFilter;
    for (const CycInputSource& src : pFilter->getInputSources())
    {
        if (src.pCycFilter->getOutputDataType() == CyC_IMAGE)
        {
            keyOverlayFilter = src.pCycFilter->getFilterKey();
        }
    }

    CCycFilterBase* pOverlayFilter;
    m_pCycCore->readFilter(keyOverlayFilter, pOverlayFilter);

    // Check if the overlay filter is the source for computing the lanes
    if (pFilter->getInputSources().size() > 0)
    {
        if (pOverlayFilter == pFilter->getInputSources()[0].pCycFilter)
            bIsSourceFilter = true;

        if (pOverlayFilter != nullptr)
            bImageRead = CCycCore::getCycImage2CvMat(pOverlayFilter, dst);
        else
            return false;

        const CPinholeCameraSensorModel* pSensorModel = static_cast<const CPinholeCameraSensorModel*>(pOverlayFilter->getSensorModel());

        if (pSensorModel != nullptr)
        {
            for (const auto& pose : poses)
            {
                std::string str_id = "id=" + std::to_string(pose.getID());

                CImageDisplayUtils::drawPoseCamView(pose, pSensorModel->cvK(), pSensorModel->cvD(), dst, 1, str_id);
            }
        }
        else
        {
            return false;
        }
    }
    return bDataRead && bImageRead;
}

bool CPlotFilterOutput::plotCycBBox3D(cv::Mat& dst, CCycFilterBase* pFilter, CycDatablockKey keyOverlayFilter)
{
    bool bReturn(false);
    
    return bReturn;
}

