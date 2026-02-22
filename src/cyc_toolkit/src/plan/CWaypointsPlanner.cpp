// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CWaypointsPlanner.h"
#include "math/CGeometry.h"
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

float CWaypointsPlanner::dist(float x1, float y1, float x2, float y2)
{
    return sqrtf(powf(x2 - x1, 2.f) + powf(y2 - y1, 2.f));
}

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

            pt_landmark.id = row.get<CyC_INT>((size_t)EULER_FORMAT::LANDMARK_ID);
            pt_landmark.pose.update(row.get<float>((size_t)EULER_FORMAT::X),
                row.get<float>((size_t)EULER_FORMAT::Y),
                row.get<float>((size_t)EULER_FORMAT::Z),
                row.get<float>((size_t)EULER_FORMAT::ROLL),
                row.get<float>((size_t)EULER_FORMAT::PITCH),
                row.get<float>((size_t)EULER_FORMAT::YAW));
            pt_landmark.travel_time = row.get<float>((size_t)EULER_FORMAT::TRAVEL_TIME);
            str2waypoints(row.get<std::string>((size_t)EULER_FORMAT::WAYPOINTS), pt_landmark.waypoints);
            m_Landmarks.emplace_back(pt_landmark);
        }
    }
    else if (csv_waypoints.get_column_names().size() == (size_t)QUAT_FORMAT::NUM)
    {
        while (csv_waypoints.next_row())
        {
            CycLandmark pt_landmark;
            const csv::reader::row& row = csv_waypoints.get_row();

            pt_landmark.id = row.get<CyC_INT>((size_t)QUAT_FORMAT::LANDMARK_ID);
            pt_landmark.pose.update(row.get<float>((size_t)QUAT_FORMAT::X),
                row.get<float>((size_t)QUAT_FORMAT::Y),
                row.get<float>((size_t)QUAT_FORMAT::Z),
                row.get<float>((size_t)QUAT_FORMAT::Qx),
                row.get<float>((size_t)QUAT_FORMAT::Qy),
                row.get<float>((size_t)QUAT_FORMAT::Qz),
                row.get<float>((size_t)QUAT_FORMAT::Qw));
            pt_landmark.travel_time = row.get<float>((size_t)QUAT_FORMAT::TRAVEL_TIME);
            str2waypoints(row.get<std::string>((size_t)QUAT_FORMAT::WAYPOINTS), pt_landmark.waypoints);
            m_Landmarks.emplace_back(pt_landmark);
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
    m_Landmarks = refs;
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
        CsvWritter << name << "," << landmark.id << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << rot.x() << "," << rot.y() << "," << rot.z() << "," << rot.w() << "," << landmark.travel_time << ",";

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

bool CWaypointsPlanner::getLandmark(const CyC_INT _marker_id, CycLandmark& _landmark)
{
    for (const auto& l : m_Landmarks)
    {
        if (l.id == _marker_id)
        {
            _landmark = m_Landmarks[_marker_id];
            return true;
        }
    }
    return false;
}

bool CWaypointsPlanner::getWaypoints(const CyC_INT _marker_id, std::vector<Eigen::Vector4f>& _waypoints)
{
    _waypoints.clear();
    for (const auto& l : m_Landmarks)
    {
        if (l.id == _marker_id)
        {
            _waypoints = l.waypoints;
            return true;
        }
    }
    return false;
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

    for (auto it = tmp_waypoints.begin(); it != tmp_waypoints.end(); ++it)
        _waypoints.emplace_back(*it);
}

bool CWaypointsPlanner::addLandmark(const int& _id, const CPose& _pose, const std::vector<Eigen::VectorXf>& _waypoints)
{
    for (const auto& l : m_Landmarks)
        if (l.id == _id)
            return false;

    CycLandmark landmark;
    landmark.id = _id;
    for (const auto& waypt : _waypoints)
    {
        if (waypt.size() < 4)
            return false;
        landmark.waypoints.emplace_back(waypt.head<4>());
    }
    m_Landmarks.emplace_back(landmark);

    return true;
}

void CWaypointsPlanner::landmark2mission(const CycLandmark& _landmark, std::vector<Eigen::Vector4f>& _mission_pts)
{
    std::vector<Eigen::Vector2f> waypoints;
    //for (const auto& w : _landmark.waypoints)
    //    waypoints.emplace_back(w.x(), w.y());
    for (auto it = _landmark.waypoints.rbegin(); it != _landmark.waypoints.rend(); ++it)
    {
        const auto& w = *it;
        waypoints.emplace_back(w.x(), w.y());
    }
    waypoints.emplace_back(_landmark.pose.translation_3x1().x(), _landmark.pose.translation_3x1().y());

    waypoints2mission(waypoints, _mission_pts);
}

void CWaypointsPlanner::waypoints2mission(const std::vector<Eigen::Vector2f>& waypoints, std::vector<Eigen::Vector4f>& mission_pts)
{
    mission_pts.clear();
    const bool interpolate_points = true;
    const float velocity = 1.F;  // Reference velocity
    float yaw = 0.F;       // Default yaw

    for (const auto& w : waypoints)
    {
        float x = w.x(), y = w.y();

        if (mission_pts.size() > 0)
        {
            const auto prev_x = mission_pts.back()[0];
            const auto prev_y = mission_pts.back()[1];

            // Estimate yaw angle for each state
            yaw = atan2f(y - prev_y, x - prev_x);

            // Interpolate points in between measurements
            if (interpolate_points)
            {
                const auto prev_angle = mission_pts.back()[3];
                const Eigen::Vector4f begin{ prev_x, prev_y, velocity, prev_angle };
                const Eigen::Vector4f end{ x, y, velocity, yaw };
                interpolate_between_points(begin, end, mission_pts);
            }
        }

        mission_pts.emplace_back(x, y, velocity, yaw);
    }
}

bool CWaypointsPlanner::waypoints2mission(csv::reader& csv_reader, std::vector<Eigen::Vector4f>& _out_ref_path_pts)
{
    std::vector<Eigen::Vector2f> pts;

    csv_reader.select_cols("x", "y", "waypoints");
    float x, y;
    std::string waypoints;
    float x_wpt, y_wpt;
    std::string delimiter_outside = "[";
    std::string delimiter_inside = ";";
    while (csv_reader.read_row(x, y, waypoints))
    {
        // Insert landmark to the reference set points vector
        pts.emplace_back(x, y);

        // Remove first two "["
        waypoints = waypoints.substr(2, waypoints.size() - 1);

        // Replace trailing "]" with delimiter "["
        waypoints[waypoints.size() - 1] = '[';

        // Split waypoints
        unsigned long long pos = 0;
        std::string waypoint;
        while ((pos = waypoints.find(delimiter_outside)) != std::string::npos)
        {
            waypoint = waypoints.substr(0, pos);

            // Replace trailing "]" with delimiter ";"
            waypoint[waypoint.size() - 1] = ';';

            unsigned long long pos2 = 0;
            std::string wpt_element;
            CyC_INT i = 0;
            while ((pos2 = waypoint.find(delimiter_inside)) != std::string::npos)
            {
                wpt_element = waypoint.substr(0, pos2);

                if (i == 0)
                {
                    x_wpt = std::stof(wpt_element);
                }
                if (i == 1)
                {
                    y_wpt = std::stof(wpt_element);
                }
                i++;
                waypoint.erase(0, pos2 + delimiter_inside.length());
            }

            pts.emplace_back(x_wpt, y_wpt);

            waypoints.erase(0, pos + delimiter_outside.length());
        }
    }

    waypoints2mission(pts, _out_ref_path_pts);

    return true;
}

void CWaypointsPlanner::interpolate_between_points(const Eigen::Vector4f& begin, const Eigen::Vector4f& end, std::vector<Eigen::Vector4f>& _in_out_ref_path_pts)
{
    const float sampling_distance = 0.1f;  // If points will be interpolated, use this distance

    const auto prev_x = begin[0];
    const auto prev_y = begin[1];

    const auto x = end[0];
    const auto y = end[1];
    const auto v = end[2];
    const auto yaw = end[3];

    if (!_in_out_ref_path_pts.empty())
    {
#ifdef USE_OLD_METHOD // Uses first degree polynomials. Can't interpolate if slope is infinite
        const float m = (y - prev_y) / (x - prev_x);
        if (abs(x - prev_x) < FLT_EPSILON)
            return;

        const float b = y - m * x;
        const float d = dist(x, y, prev_x, prev_y);
        if (d > sampling_distance)
        {
            const CyC_INT num_interpolate = (CyC_INT)(d / sampling_distance);
            for (CyC_INT interp_idx = 0; interp_idx < num_interpolate; ++interp_idx)
            {
                const float new_x = prev_x + interp_idx * ((x - prev_x) / num_interpolate);
                const float new_y = m * new_x + b;
                const float new_yaw = atan2f(new_y - prev_y, new_x - prev_x);
                _in_out_ref_path_pts.emplace_back(new_x, new_y, v, new_yaw);
            }
        }
#else
        const float d = dist(x, y, prev_x, prev_y);
        const CyC_INT num_interpolate = (CyC_INT)ceil(d / sampling_distance);
        if (num_interpolate > 0)
        {
            const float dx = (x - prev_x) / num_interpolate;
            const float dy = (y - prev_y) / num_interpolate;
            float new_x = prev_x;
            float new_y = prev_y;
            for (CyC_INT interp_idx = 0; interp_idx < num_interpolate; ++interp_idx)
            {
                new_x += dx;
                new_y += dy;

                const float new_yaw = atan2f(new_y - prev_y, new_x - prev_x);
                _in_out_ref_path_pts.emplace_back(new_x, new_y, v, new_yaw);
            }
        }
#endif
    }
}

CycLandmark CWaypointsPlanner::getClosestTraj(const CycLandmarks& _landmarks, const CPose& _dst)
{
    float fMinEd = std::numeric_limits<float>::max();
    CycLandmark ref_traj;
    Eigen::Vector2f destination = _dst.translation_3x1().head<2>();
    for (const auto& traj : _landmarks)
    {
        Eigen::Vector2f landmark = traj.pose.translation_3x1().head<2>();

        float fEd = CGeometry::euclidean_dist(destination, landmark);
        if (fEd < fMinEd)
        {
            ref_traj = traj;
            fMinEd = fEd;
        }
    }
    return ref_traj;
}
