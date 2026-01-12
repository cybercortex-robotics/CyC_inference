// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CWaypointsPlanner.h"
#include <iostream>
#include <sstream>
#include <algorithm>

CWaypointsPlanner::CWaypointsPlanner()
{}

CWaypointsPlanner::CWaypointsPlanner(std::string _waypoints_file)
{
    loadWaypoints(_waypoints_file);
}

CWaypointsPlanner::~CWaypointsPlanner()
{}

bool CWaypointsPlanner::loadWaypoints(const std::string& _waypoints_file)
{
    m_Landmarks.clear();

    // Check if file exists
    if (!CFileUtils::FileExist(_waypoints_file.c_str()))
    {
        spdlog::error("CWaypointsPlanner::loadWaypoints(): Waypoints file not found.");
        return false;
    }

    csv::reader csv_waypoints;
    if (!csv_waypoints.open(_waypoints_file))
    {
        spdlog::error("CWaypointsPlanner::loadWaypoints(): Failed to open csv \"{}\"", _waypoints_file);
        return false;
    }

    enum class EULER_FORMAT { NAME, LANDMARK_ID, X, Y, Z, ROLL, PITCH, YAW, TRAVEL_TIME, WAYPOINTS, NUM };
    enum class QUAT_FORMAT  { NAME, LANDMARK_ID, X, Y, Z, Qx, Qy, Qz, Qw, TRAVEL_TIME, WAYPOINTS, NUM };
    if (csv_waypoints.get_column_names().size() == (size_t)EULER_FORMAT::NUM)
    {
        while (csv_waypoints.next_row())
        {
            CycLandmark pt_landmark;
            const csv::reader::row& row = csv_waypoints.get_row();

            CyC_INT marker_id = row.get<CyC_INT>((size_t)EULER_FORMAT::LANDMARK_ID);
            pt_landmark.pose.update(row.get<float>((size_t)EULER_FORMAT::X),
                row.get<float>((size_t)EULER_FORMAT::Y),
                row.get<float>((size_t)EULER_FORMAT::Z),
                row.get<float>((size_t)EULER_FORMAT::ROLL),
                row.get<float>((size_t)EULER_FORMAT::PITCH),
                row.get<float>((size_t)EULER_FORMAT::YAW));
            pt_landmark.travel_time = row.get<float>((size_t)EULER_FORMAT::TRAVEL_TIME);
            str2waypoints(row.get<std::string>((size_t)EULER_FORMAT::WAYPOINTS), pt_landmark.waypoints);

            m_Landmarks.emplace(marker_id, pt_landmark);
        }
    }
    else if (csv_waypoints.get_column_names().size() == (size_t)QUAT_FORMAT::NUM)
    {
        while (csv_waypoints.next_row())
        {
            CycLandmark pt_landmark;
            const csv::reader::row& row = csv_waypoints.get_row();

            CyC_INT marker_id = row.get<CyC_INT>((size_t)QUAT_FORMAT::LANDMARK_ID);
            pt_landmark.pose.update(row.get<float>((size_t)QUAT_FORMAT::X),
                row.get<float>((size_t)QUAT_FORMAT::Y),
                row.get<float>((size_t)QUAT_FORMAT::Z),
                row.get<float>((size_t)QUAT_FORMAT::Qx),
                row.get<float>((size_t)QUAT_FORMAT::Qy),
                row.get<float>((size_t)QUAT_FORMAT::Qz),
                row.get<float>((size_t)QUAT_FORMAT::Qw));
            pt_landmark.travel_time = row.get<float>((size_t)QUAT_FORMAT::TRAVEL_TIME);
            str2waypoints(row.get<std::string>((size_t)QUAT_FORMAT::WAYPOINTS), pt_landmark.waypoints);

            m_Landmarks.emplace(marker_id, pt_landmark);
        }
    }
    else
    {
        spdlog::error("CWaypointsPlanner::loadWaypoints(): Unknown csv format: wrong columns count");
    }

    return true;
}

bool CWaypointsPlanner::loadWaypoints(const CycLandmarks& refs)
{
    m_Landmarks.clear();
    for (const auto& l : refs)
    {
        m_Landmarks[l.first].pose = l.second.pose;
        m_Landmarks[l.first].travel_time = l.second.travel_time;

        for (auto it = l.second.waypoints.rbegin(); it != l.second.waypoints.rend(); ++it)
            m_Landmarks[l.first].waypoints.emplace_back(*it);
    }

    return true;
}

bool CWaypointsPlanner::saveWaypoints(CycLandmarks& _landmarks, const std::string& _waypoints_file)
{
    std::ofstream CsvWritter(_waypoints_file);

    CsvWritter << "name,landmark_id,x,y,z,qx,qy,qz,qw,travel_time,waypoints" << std::endl;

    for (CyC_INT i = 0; i < _landmarks.size(); ++i)
    {
        CycLandmark& landmark = _landmarks[i];
        std::string name = "Mission " + std::to_string(i);
        Eigen::Vector3f pos = landmark.pose.translation_3x1();
        Eigen::Vector4f rot = landmark.pose.rotation_quat().to_vector();
        CsvWritter << name << "," << i << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << rot.x() << "," << rot.y() << "," << rot.z() << "," << rot.w() << "," << landmark.travel_time << ",";

        CsvWritter << "[";
        for (CyC_INT j = 0; j < landmark.waypoints.size(); j++)
        {
            auto w = landmark.waypoints[j];
            CsvWritter << "[" << w.x() << ";" << w.y() << ";" << w.z() << ";0]";
        }
        CsvWritter << "]";

        CsvWritter << std::endl;
    }

    CsvWritter.flush();

    return false;
}

bool CWaypointsPlanner::getMarker(const CyC_INT _marker_id, CycLandmark& _landmark)
{
    if (m_Landmarks.find(_marker_id) == m_Landmarks.end())
    {
        return false;
    }
    else
    {
        _landmark = m_Landmarks[_marker_id];
        return true;
    }
}

bool CWaypointsPlanner::getLandmark(const CyC_INT _marker_id, CycLandmark& _waypoints)
{
    if (m_Landmarks.find(_marker_id) == m_Landmarks.end())
        return false;

    _waypoints = m_Landmarks[_marker_id];

    return true;
}

bool CWaypointsPlanner::getWaypoints(const CyC_INT _marker_id, std::vector<Eigen::Vector4f>& _waypoints)
{
    if (m_Landmarks.find(_marker_id) == m_Landmarks.end())
        return false;

    _waypoints.clear();
    _waypoints = m_Landmarks[_marker_id].waypoints;

    return true;
}

CyC_UINT CWaypointsPlanner::getNumLandmarks()
{
    return static_cast<CyC_UINT>(m_Landmarks.size());
}

void CWaypointsPlanner::str2waypoints(std::string _str_waypoints, std::vector<Eigen::Vector4f>& _waypoints)
{
    //_waypoints.clear();

    std::vector<Eigen::Vector4f> tmp_waypoints;
    std::vector<std::string> result;
    std::stringstream ss_items(_str_waypoints);
    std::string item;

    // Parse 3D points entries
    while (std::getline(ss_items, item, '['))
    {
        item.erase(std::remove(item.begin(), item.end(), ']'), item.end());

        // Parse the X, Y, Z values
        std::stringstream ss_values(item);
        std::string waypt_values;
        CyC_INT idx = 0;
        Eigen::Vector4f pt3d = Eigen::Vector4f::Ones();
        while (std::getline(ss_values, waypt_values, ';'))
        {
            switch (idx)
            {
            case 0:
                pt3d.x() = std::stof(waypt_values);
                break;
            case 1:
                pt3d.y() = std::stof(waypt_values);
                break;
            case 2:
                pt3d.z() = std::stof(waypt_values);
                break;
            case 3:
                pt3d.w() = std::stof(waypt_values);
                break;
            }
            ++idx;
        }

        if (idx == 4)
            tmp_waypoints.emplace_back(pt3d);
    }

    for (auto it = tmp_waypoints.rbegin(); it != tmp_waypoints.rend(); ++it)
        _waypoints.emplace_back(*it);
}
