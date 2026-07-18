// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CyC_TYPES.h"
#include <os/CCsvReader.h>
#include "os/CFileUtils.h"

#ifndef CWaypointsPlanner_H_
#define CWaypointsPlanner_H_

class CWaypointsPlanner
{
public:
    CWaypointsPlanner();
    CWaypointsPlanner(std::string _waypoints_file);
    ~CWaypointsPlanner();

    bool        loadWaypoints(const std::string& _waypoints_file);
    bool        loadWaypoints(const CycLandmarks& refs);
    static bool saveWaypoints(CycLandmarks& _landmarks, const std::string& _waypoints_file);

    bool        addLandmark(const int& _id, const CPose& _pose, const std::vector<Eigen::VectorXf>& _waypoints);

    const       CycLandmarks getLandmarks() const { return m_Landmarks; };
    bool        getLandmark(const CyC_INT _marker_id, CycLandmark& _landmark);
    bool        getWaypoints(const CyC_INT _marker_id, std::vector<CycSetPoint>& _waypoints);
    CyC_UINT    getNumLandmarks();

    static void landmark2mission(const CycLandmark& _landmark, std::vector<CycSetPoint>& _mission_pts);
    static void waypoints2mission(const std::vector<CycSetPoint>& waypoints, std::vector<CycSetPoint>& mission_pts);
    static bool waypoints2mission(csv::reader& csv_reader, std::vector<CycSetPoint>& _out_ref_path_pts);

    static void interpolate_between_points(const CycSetPoint& begin, const CycSetPoint& end, std::vector<CycSetPoint>& _in_out_ref_path_pts);
    static CycLandmark getClosestTraj(const CycLandmarks& _landmarks, const CPose& _dst);

private:
    void str2waypoints(std::string _str_waypoints, std::vector<CycSetPoint>& _waypoints);
    static float dist(float x1, float y1, float x2, float y2);

private:
    CycLandmarks m_Landmarks;
};

#endif /* CWaypointsPlanner_H_ */
