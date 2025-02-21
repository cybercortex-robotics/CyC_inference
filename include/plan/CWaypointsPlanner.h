// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CCR_TYPES.h"
#include <csv_reader.h>
#include "os/CFileUtils.h"

#ifndef CWaypointsPlanner_H_
#define CWaypointsPlanner_H_

class CWaypointsPlanner
{
public:
    CWaypointsPlanner();
    CWaypointsPlanner(std::string _waypoints_file);
    ~CWaypointsPlanner();

    bool        loadWaypoints(std::string _waypoints_file);
    bool        loadWaypoints(const CcrLandmarks& refs);
    bool        getMarker(const CCR_INT _marker_id, CcrLandmark& _landmark);
    void        getMap(std::unordered_map<CCR_INT, CcrLandmark>& _map);
    bool        getLandmark(const CCR_INT _marker_id, CcrLandmark& _waypoints);
    bool        getWaypoints(const CCR_INT _marker_id, std::vector<Eigen::Vector4f>& _waypoints);
    CCR_UINT    getNoWaypoints();

private:
    void str2waypoints(std::string _str_waypoints, std::vector<Eigen::Vector4f>& _waypoints);

private:
    CcrLandmarks m_Landmarks;
};

#endif /* CWaypointsPlanner_H_ */