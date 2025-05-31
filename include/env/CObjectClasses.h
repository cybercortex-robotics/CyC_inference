// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CObjectClasses_H_
#define CObjectClasses_H_

#include "CyC_TYPES.h"
#include "os/CFileUtils.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)

class color
{
public:
    static const cv::Scalar colormap[155];

    // OpenCV color channel order is BGR[A]
    static cv::Scalar black;
    static cv::Scalar gray;
    static cv::Scalar white;
    static cv::Scalar red;
    static cv::Scalar green;
    static cv::Scalar blue;
    static cv::Scalar yellow;
    static cv::Scalar pink;

    static cv::Scalar x_axis;
    static cv::Scalar y_axis;
    static cv::Scalar z_axis;

    static cv::Scalar undefined;
    static cv::Scalar vehicle;
    static cv::Scalar mission_path;
    static cv::Scalar reference_path;
    static cv::Scalar current_path;
    static cv::Scalar trajectories_samples;
    static cv::Scalar free_space;
    static cv::Scalar obstacle;
    static cv::Scalar obstacle_prediction;
    static cv::Scalar road_model;
    static cv::Scalar lidar;
    static cv::Scalar ultrasonics;
    static cv::Scalar depth;
    static cv::Scalar slam_map_points;
    static cv::Scalar wheel_stuck;

    static cv::Scalar info;
    static cv::Scalar solutions;
    static cv::Scalar best_solution;
    static cv::Scalar curr_pts_2d;
    static cv::Scalar prev_pts_2d;
    static cv::Scalar reprojection;
    static cv::Scalar orthogonal_projection;
    static cv::Scalar reproj_pts_2d_pos_depth;
    static cv::Scalar reproj_pts_2d_neg_depth;
    static cv::Scalar epi_lines;
};

class CObjectClasses
{
public:
    enum STATIC_OBJECT_CLASSES
    {
        UNDEFINED       = -1,
        LIDAR           = -2,
        ULTRASONICS     = -3,
        DEPTH           = -4,
        SLAM_MAP_POINTS = -5
    };

private:
    void addStaticClasses()
    {
        m_ObjectClassesMap[UNDEFINED]       = "Undefined";
        m_ObjectClassesMap[LIDAR]           = "Lidar";
        m_ObjectClassesMap[ULTRASONICS]     = "Ultrasonics";
        m_ObjectClassesMap[DEPTH]           = "Depth";
        m_ObjectClassesMap[SLAM_MAP_POINTS] = "Map points";
    }

public:
    CObjectClasses(const std::string& _object_classes_file);
    virtual ~CObjectClasses();

    bool isInitialized()                                { return m_bIsInitialized; };
    CyC_UINT getNumClasses()                            { return m_nNumClasses; };
    std::unordered_map<CyC_INT, std::string> getLandmarks()   { return m_ObjectClassesMap; };

    static cv::Scalar getColor(const CyC_INT& _object_class);

private:
    bool m_bIsInitialized = false;
    CyC_UINT m_nNumClasses = 0;

    std::unordered_map<CyC_INT, std::string> m_ObjectClassesMap;
};

#endif /* CObjectClasses_H_ */
