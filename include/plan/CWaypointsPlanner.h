// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CyC_TYPES.h"
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

    bool        loadWaypoints(const std::string& _waypoints_file);
    bool        loadWaypoints(const CycLandmarks& refs);
    static bool saveWaypoints(CycLandmarks& _landmarks, const std::string& _waypoints_file);

    const std::unordered_map<CyC_INT, CycLandmark>& getLandmarks() const { return m_Landmarks; };
    bool        getLandmark(const CyC_INT _marker_id, CycLandmark& _waypoints);
    bool        getMarker(const CyC_INT _marker_id, CycLandmark& _landmark);
    bool        getWaypoints(const CyC_INT _marker_id, std::vector<Eigen::Vector4f>& _waypoints);
    CyC_UINT    getNumLandmarks();

private:
    void str2waypoints(std::string _str_waypoints, std::vector<Eigen::Vector4f>& _waypoints);

private:
    CycLandmarks m_Landmarks;
};

#endif /* CWaypointsPlanner_H_ */