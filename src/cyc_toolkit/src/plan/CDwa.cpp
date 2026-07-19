// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <limits>
#include <math.h>
#include "CDwa.h"
#include "math/CPolynomialFitting.h"
#include <env/CConcaveHull.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

CDwa::CDwa(const float _dt,
    const std::string& _vehicle_model_file,
    const float _lookahead_distance,
    const float& _goal_distance,
    const std::vector<CyC_INT>& _traversable_class_ids) :
    m_GoalDistance(_goal_distance),
    m_TravesableClassIDs(_traversable_class_ids)
{
    m_pVehicleModel = std::make_unique<CModelVehicle>(_vehicle_model_file);
    m_config.dt = _dt;
    m_config.angular_momentum_coefficient = 1.f;
    m_config.linear_momentum_coefficient = 1.f;
    m_config.top_speed_coefficient = 1.f;
    m_config.goal_coefficient = 5.f;
    m_config.lanes_coefficient = 5.f;
    m_config.lookahead_distance = _lookahead_distance;
    m_config.orientation_coefficient = 1.f;
    m_config.traversable_coefficient = 100.f;

    m_PreviousControl.u.resize(2);
    m_PreviousControl.u << 0.f, 0.f;

    m_bEnabled = true;
}

void CDwa::setMissionPath(const std::vector<CycSetPoint>& _mission_path, const CycState& _vehicle_state)
{
    m_MissionPath = _mission_path;

    // Find a mean distance between sampling points from global trajectory
    float sum = 0.f;
    auto dist = [&](const Eigen::Vector4f& pt1, const Eigen::Vector4f& pt2) {
        return sqrtf(powf(pt2.x() - pt1.x(), 2.f) + powf(pt2.y() - pt1.y(), 2.f));
    };

    for (size_t idx = 1; idx < m_MissionPath.size(); ++idx)
    {
        sum += dist(m_MissionPath[idx - 1].r, m_MissionPath[idx].r);
    }

    const auto mean_pt_dist = sum / m_MissionPath.size();
    const auto goal_pt_dist = m_config.lookahead_distance / m_NumGoalPoints;

    // Find number of points from global trajectory to match goal point distance
    const auto num_pts_per_goal_point = (CyC_UINT)(goal_pt_dist / mean_pt_dist);

    // Find index up until to search for next goal point during control phase
    m_ClosestIndex = m_NumGoalPoints * num_pts_per_goal_point;

    float minDist = std::numeric_limits<float>::max();
    const Eigen::Vector4f state{ _vehicle_state.x_hat[0], _vehicle_state.x_hat[1], 0.F, 0.F };
    for (size_t idx = 0; idx < m_MissionPath.size(); ++idx)
    {
        auto d = dist(m_MissionPath[idx].r, state);
        if (d < minDist)
        {
            minDist = d;
            m_PreviousTrajectoryPointIndex = idx;
        }
    }

    //spdlog::info("{}, {}", m_MissionPath[m_PreviousTrajectoryPointIndex].x(), m_MissionPath[m_PreviousTrajectoryPointIndex].y());
    //spdlog::info("{}, {}", _vehicle_state.x_hat.x(), _vehicle_state.y());
}

void CDwa::predictState(float vel, float steer)
{
    Eigen::VectorXf ctrl;
    ctrl.resize(2);
    ctrl << vel, steer;
    m_pVehicleModel->step(m_config.dt, ctrl);
}

bool CDwa::obstacleCollision(const Eigen::VectorXf& state, const Eigen::VectorXf& obstacle_point)
{
    // Give the robot mannouver space
    const float max_dim = std::max(m_pVehicleModel->m_fVehicleLength, m_pVehicleModel->m_fVehicleWidth);
    const float vehicle_x1 = state(0) - max_dim / 2.F;
    const float vehicle_y1 = state(1) - max_dim / 2.F;
    const float vehicle_x2 = state(0) + max_dim / 2.F;
    const float vehicle_y2 = state(1) + max_dim / 2.F;

    const float x = obstacle_point(0);
    const float y = obstacle_point(1);

    return (x > vehicle_x1) && (x < vehicle_x2) && (y > vehicle_y1) && (y < vehicle_y2);
    //return sqrtf(powf(x - state(0), 2.F) + powf(y - state(1), 2.F)) < max_dim;
}

// Cost functions
float CDwa::obstacleCost(const std::vector<CycSetPoint>& trajectory, const std::vector<CycSetPoint>& obstacles)
{
    const float obstacle_cost = 10e6f;

    if (!obstacles.empty())
    {
        for (const CycSetPoint& pt : trajectory)
        {
            const Eigen::Vector2f xy(pt.r.x(), pt.r.y());

            for (const CycSetPoint& ob : obstacles)
            {
                if (obstacleCollision(xy, ob.r))
                {
                    return obstacle_cost;
                }
            }
        }
    }

    return 0.f;
}

float CDwa::traversableCost(const std::vector<CycSetPoint>& trajectory, const std::vector<CycSetPoint>& traversable)
{
    float traversable_cost = 10e6f;

    if (!traversable.empty())
    {
        CyC_INT num_nodes = 0;

        for (const CycSetPoint& pt : trajectory)
        {
            const Eigen::Vector2f xy(pt.r.x(), pt.r.y());

            for (const CycSetPoint& ob : traversable)
            {
                if (obstacleCollision(xy, ob.r))
                    ++num_nodes;
            }
        }

        traversable_cost = 1.f / (float)num_nodes;
    }

    return traversable_cost;
}

float CDwa::goalPointsCost(const std::vector<CycSetPoint>& trajectory, const std::vector<CycSetPoint>& goal_points)
{
    float cost = 0.F;

    if (!trajectory.empty())
    {
        for (const CycSetPoint& goal_point : goal_points)
        {
            std::vector<float> distances;
            distances.reserve(trajectory.size());

            std::transform(trajectory.begin(), trajectory.end(), std::back_inserter(distances),
                [this, &goal_point](const CycSetPoint& pt) { return distance(pt.r, goal_point.r); });

            const auto it = std::min_element(distances.begin(), distances.end());
            cost += *it;
        }
    }

    return cost;
}

float CDwa::angularMomentumCost(float angular_vel)
{
    auto ang_vel_prev = m_PreviousControl.u[1];
    return fabs(angular_vel - ang_vel_prev);
}

float CDwa::linearMomentumCost(float linear_vel)
{
    auto lin_vel_prev = m_PreviousControl.u[0];
    return fabs(linear_vel - lin_vel_prev);
}

float CDwa::topSpeedCost(float linear_vel)
{
    if (linear_vel < 0.f)
        return fabs(m_pVehicleModel->m_fMaxReverseSpeed - linear_vel);

    return m_pVehicleModel->m_fMaxForwardSpeed - linear_vel;
}

float CDwa::lanesCost(const std::vector<CycSetPoint>& trajectory, const CycLanesModel& lanes)
{
    // no lanes available
    if (lanes.empty())
        return 0.F;

    CycLane leftLane = lanes.back();
    CycLane rightLane = lanes.back();

    // find the closest left and right lanes
    for (auto& lane : lanes)
    {
        if ((lane.model[0] < 0.F) &&
            (abs(lane.model[0]) < abs(rightLane.model[0])))
        {
            rightLane = lane;
        }
        else if ((lane.model[0] > 0.F) &&
                 (abs(lane.model[0]) < abs(leftLane.model[0])))
        {
            leftLane = lane;
        }
    }

    // if the left and right lane are the same, ignore them?
    if (leftLane.id == rightLane.id)
        return 0.F;

    // generate trajectory and use the already-implemented function
    // to calculate the cost of staying between lanes
    std::vector<CycSetPoint> lane_trajectory;
    lane_trajectory.reserve(trajectory.size());
    for (const auto& point : trajectory)
    {
        const auto y =
            (CPolynomialFitting::polyeval(leftLane.model, point.r.x()) +
             CPolynomialFitting::polyeval(rightLane.model, point.r.x())) * 0.5F;

        CycSetPoint pt(2); pt.r << point.r.x(), y;
        lane_trajectory.emplace_back(pt);
    }

    return goalPointsCost(trajectory, lane_trajectory);
}

float CDwa::roadSegmentationCost(const CycTrajectory& trajectory, const std::vector<Eigen::VectorXf>& traversable_nodes)
{
    const CPose robot_pose{ trajectory.front().x(), trajectory.front().y(), 0.F, 0.F, 0.F, trajectory.front()[3] };
    const Eigen::Matrix4f T = robot_pose.transform().inverse();

    CyC_INT num_non_collisions = 0;
    if (!traversable_nodes.empty() && !trajectory.empty())
    {
        for (const Eigen::VectorXf& pt : trajectory)
        {
            const Eigen::Vector4f pt_w{ pt.x(), pt.y(), 0.F, 1.F };
            const Eigen::Vector4f pt_veh = T * pt_w;

            Eigen::VectorXf xyz(2);
            xyz << pt_veh.x(), pt_veh.y();

            for (const Eigen::VectorXf& traversable_node : traversable_nodes)
            {
                // If the robot is not on the segmented road, the cost is very high
                if (!obstacleCollision(xyz, traversable_node))
                {
                    ++num_non_collisions;
                }
            }
        }

        return ((float)num_non_collisions) / (trajectory.size() * traversable_nodes.size());
    }

    return 0.F;
}

bool onSegment(const Eigen::Vector4f& p, const Eigen::Vector4f& q, const Eigen::Vector4f& r)
{
    return 
        ((q.x() <= std::max(p.x(), r.x())) &&
        (q.x() >= std::min(p.x(), r.x())) &&
        (q.y() <= std::max(p.y(), r.y())) &&
        (q.y() >= std::min(p.y(), r.y())));
}

CyC_INT orientation(const Eigen::Vector4f& p, const Eigen::Vector4f& q, const Eigen::Vector4f& r)
{
    const auto val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
    if (fabsf(val) < 1e-6F)
        return 0;

    return (val > 0.F) ? 1 : 2;
}

bool doIntersect(const Eigen::Vector4f& p1, const Eigen::Vector4f& q1, const Eigen::Vector4f& p2, const Eigen::Vector4f& q2)
{
    const auto o1 = orientation(p1, q1, p2);
    const auto o2 = orientation(p1, q1, q2);
    const auto o3 = orientation(p2, q2, p1);
    const auto o4 = orientation(p2, q2, q1);

    if ((o1 != o2) && (o3 != o4))
    {
        return true;
    }

    if ((o1 == 0) && onSegment(p1, p2, q1))
        return true;

    if ((o2 == 0) && onSegment(p1, q2, q1))
        return true;

    if ((o3 == 0) && onSegment(p2, p1, q2))
        return true;

    if ((o4 == 0) && onSegment(p2, q1, q2))
        return true;

    return false;
}

std::vector<Eigen::Vector4f> getVehicleCorners(const Eigen::Vector4f& center, float max_dim)
{
    const float vehicle_x1 = center(0) - max_dim / 2.F;
    const float vehicle_y1 = center(1) - max_dim / 2.F;
    const float vehicle_x2 = center(0) + max_dim / 2.F;
    const float vehicle_y2 = center(1) + max_dim / 2.F;

    std::vector<Eigen::Vector4f> corners;
    corners.emplace_back(vehicle_x1, vehicle_y1, 0.F, 1.F);
    corners.emplace_back(vehicle_x1, vehicle_y2, 0.F, 1.F);
    corners.emplace_back(vehicle_x2, vehicle_y1, 0.F, 1.F);
    corners.emplace_back(vehicle_x2, vehicle_y2, 0.F, 1.F);

    return corners;
}

float CDwa::roadSegmentationHullCost(const CycTrajectory& trajectory, const std::vector<Eigen::Vector4f>& hull_nodes)
{
    const CPose robot_pose{ trajectory.front().x(), trajectory.front().y(), 0.F, 0.F, 0.F, trajectory.front()[3] };
    const Eigen::Matrix4f T = robot_pose.transform().inverse();

    const float max_dim = std::min(m_pVehicleModel->m_fVehicleLength, m_pVehicleModel->m_fVehicleWidth);

    if (!trajectory.empty() && (hull_nodes.size() > 2))
    {
        size_t num_points_inside = 0;
        for (const auto& pt : trajectory)
        {
            const Eigen::Vector4f pt_w{ pt.x(), pt.y(), 0.F, 1.F };
            const Eigen::Vector4f pt_veh = T * pt_w;

            const auto vehicle_corners = getVehicleCorners(pt_veh, max_dim);
            bool vehicle_inside = true;

            for (const auto& corner : vehicle_corners)
            {
                const Eigen::Vector4f extreme_point{ 1e6F, corner.y(), 0.F, 1.F };

                size_t i = 0;
                size_t count = 0;
                do
                {
                    const size_t next = (i + 1) % hull_nodes.size();
                    if (doIntersect(hull_nodes[i], hull_nodes[next], corner, extreme_point))
                    {
                        if ((orientation(hull_nodes[i], corner, hull_nodes[next]) == 0))
                        {
                            count = onSegment(hull_nodes[i], corner, hull_nodes[next]) ? 1 : 2;
                            break;
                        }

                        ++count;
                    }

                    i = next;
                } while (i != 0);

                vehicle_inside &= (count % 2) == 1;
            }

            num_points_inside += vehicle_inside ? 1 : 0;
        }

        return (1.F - (float)num_points_inside / trajectory.size());
    }

    // What does it mean if there are not enough hull nodes?
    // Bad detection? Bad projection? Bad calculated hull?
    // All cases should be ignored from the cost?
    return 0.F;
}

std::vector<CycSetPoint> CDwa::generateTrajectory(float target_speed, float steer)
{
    std::vector<CycSetPoint> traj;

    // Sanity check
    float speed = target_speed;
    float fPredictTime;
    if (fabs(speed) < 0.1f)
        fPredictTime = m_config.lookahead_distance / 0.1f;
    else
        fPredictTime = m_config.lookahead_distance / fabs(target_speed);

    traj.reserve((CyC_UINT)(fPredictTime / m_config.dt + 10));

    for (auto t = 0.f; t <= fPredictTime; t += m_config.dt)
    {
        predictState(target_speed, steer);
        CycSetPoint sp;
        sp.r = m_pVehicleModel->x();
        traj.push_back(std::move(sp));
    }

    return traj;
}

float get_angle(
    const Eigen::VectorXf& pt1,
    const Eigen::VectorXf& pt2)
{
    Eigen::VectorXf pt3(2);
    pt3 << 100.F, pt1.y();

    const cv::Vec2f v0{ pt2.x() - pt1.x(), pt2.y() - pt1.y() };
    const cv::Vec2f v1{ pt3.x() - pt1.x(), pt3.y() - pt1.y() };

    cv::Mat m;
    cv::hconcat(v0, v1, m);

    float angle = atan2(cv::determinant(m.t()), v0.dot(v1));

    if (angle < 0.f)
        angle += 2.f * PI;
    if (angle >= 2.f * PI)
        angle -= 2.f * PI;

    return angle;
}

void CDwa::find_goal_points(const CycState& _vehicle_state, std::vector<CycSetPoint>& _out_goal_points)
{
    _out_goal_points.clear();

    if (m_MissionPath.empty())
        return;

    const float DISTANCE_BETWEEN_POINTS = m_GoalDistance;
    Eigen::VectorXf last_point = _vehicle_state.x_hat.topRows(ModelVehicle_NumStates);

    auto distance_between = [](const Eigen::VectorXf& pt1, const Eigen::Vector4f& pt2) {
        return sqrtf(powf(pt1[0] - pt2[0], 2) + powf(pt1[1] - pt2[1], 2));
    };

    size_t index = std::min(
        CPlanningUtils::findClosestPoint(m_MissionPath, _vehicle_state, m_PreviousTrajectoryPointIndex, m_ClosestIndex),
        m_MissionPath.size() - 1);

    if (distance_between(last_point, m_MissionPath[index].r) > (2.F * DISTANCE_BETWEEN_POINTS))
    {
        index = m_PreviousTrajectoryPointIndex;
    }

    m_PreviousTrajectoryPointIndex = std::min(index, m_MissionPath.size() - 1);

    for (size_t i = 0; i < m_NumGoalPoints; ++i)
    {
        float dist = distance_between(last_point, m_MissionPath[index].r);
        while ((dist < DISTANCE_BETWEEN_POINTS) && ((index + 1) < m_MissionPath.size()))
        {
            ++index;
            dist = distance_between(last_point, m_MissionPath[index].r);
        }

        last_point << m_MissionPath[index].r.x(), m_MissionPath[index].r.y(), m_MissionPath[index].r[2], m_MissionPath[index].r[3], 0.f;
        CycSetPoint goal_sp;
        goal_sp.r = last_point;
        _out_goal_points.push_back(std::move(goal_sp));
    }
}

void CDwa::parse_octree(const CCycOcTree& _octree, std::vector<CycSetPoint>& _out_obstacles, std::vector<CycSetPoint>& _out_traversable)
{
    _out_obstacles.clear();
    _out_traversable.clear();

    auto dist = [&](const Eigen::Vector4f& pt1, const Eigen::Vector4f& pt2)
    {
        return sqrtf((pt2.x() - pt1.x()) * (pt2.x() - pt1.x()) + (pt2.y() - pt1.y()) * (pt2.y() - pt1.y()));
    };

    for (CCycOcTree::leaf_iterator it = _octree.begin_leafs(), end = _octree.end_leafs(); it != end; ++it)
    {
        //if (it->getObjectClass() < 0)
        //    continue;

        // Point in global coordinates
        CycSetPoint point(4); point.r << it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z(), 1.f;

        // Check if the segmented class is in the vector of traversable classes
        bool bTraversableFound = false;
        for (const auto& class_id : m_TravesableClassIDs)
        {
            if (it->getObjectClass() == class_id)
            {
                _out_traversable.emplace_back(point);
                bTraversableFound = true;
                break;
            }
        }

        if (!bTraversableFound)
        {
            //const float d = sqrt(pow(point.x(), 2) + pow(point.y(), 2));
            //if ((d > 0.1F) && (d < m_LookaheadDistance))
            _out_obstacles.emplace_back(point);
        }
    }
}

std::vector<Eigen::VectorXf> CDwa::find_traversable_nodes(const CCycOcTree& _octree, const CyC_INT& _traversable_class_id)
{
    std::vector<Eigen::VectorXf> traversable_nodes;

    for (CCycOcTree::leaf_iterator it = _octree.begin_leafs(), end = _octree.end_leafs(); it != end; ++it)
    {
        // Temporary conditions for the node to be a traversable node
        //if (it.getCoordinate().z() < 0.1F && it->getValue() < 0.5F)
        if (it->getObjectClass() == _traversable_class_id)
        {
            Eigen::Vector4f point;
            point << it.getCoordinate().x(),
                it.getCoordinate().y(),
                it.getCoordinate().z(),
                1.f;

            traversable_nodes.push_back(point);
        }
    }

    return traversable_nodes;
}

bool CDwa::goalPointReached(const std::vector<CycSetPoint>& goal_points, const CycState& _vehicle_state)
{
    auto dist = [&](const Eigen::VectorXf& pt1, const Eigen::VectorXf& pt2)
    {
        return sqrtf(powf(pt2.x() - pt1.x(), 2.f) + powf(pt2.y() - pt1.y(), 2.f));
    };

    const float threshold_goal_reached = 0.5f;

    // Threshold for finishing trajectory
    const ptrdiff_t tollerance = (ptrdiff_t)(0.1f * m_MissionPath.size());  // the last 10% of the trajectory marks coming close to the end-point

    // If vehicle is close to the goal point and has gone through most of the trajectory points
    if (std::distance(m_MissionPath.begin() + m_PreviousTrajectoryPointIndex, m_MissionPath.end()) < tollerance)
    {
        for (const auto& goal : goal_points)
        {
            if (dist(goal.r, m_MissionPath.back().r) < threshold_goal_reached) // If goal point is close to the end of the global trajectory
            {
                if (dist(_vehicle_state.x_hat, goal.r) < threshold_goal_reached) // If vehicle is close to the goal point
                {
                    m_GoalPointReached = true;
                    return true;
                }
            }
        }
    }
    return false;
}

/* Function for filtering control output signals; Prevents choppiness. */
CycControlInput CDwa::filterSignals(const CycControlInput& raw_control)
{
    CycControlInput ctrl = raw_control;
    if (!m_PreviousControlSignals.empty())
    {
        auto sum = 0.f;
        for (const auto& signal : m_PreviousControlSignals)
        {
            sum += signal.u[0];
        }
        ctrl.u[0] = sum / (float)m_PreviousControlSignals.size();
    }
    return ctrl;
}

CycControlInput CDwa::dwaControl(const CycState& _vehicle_state, const CycEnvironment& _env, std::vector<CycSetPoints>* _out_samples)
{
    if (_out_samples != nullptr)
        _out_samples->clear();

    CycControlInput control;
    control.u = Eigen::VectorXf::Zero(m_pVehicleModel->getNumInputs());

    std::vector<CycSetPoint> obstacles, traversable;
    parse_octree(*_env.pOccupancyModel, obstacles, traversable);
    //std::vector<Eigen::VectorXf> traversable_nodes = find_traversable_nodes(*env.pOccupancyModel, traversable_class_id);
    std::vector<CycSetPoint> goal_points;
    find_goal_points(_vehicle_state, goal_points);

    if (!m_GoalPointReached)
        if (goalPointReached(goal_points, _vehicle_state))
            return control;

    float min_cost = std::numeric_limits<float>::max();
    std::vector<CycSetPoint> best_traj;
    
    const CyC_UINT angular_vel_steps = 20;
    const CyC_UINT linear_vel_steps = 5;

    const float angular_vel_res = 2.f *  fabs(m_pVehicleModel->m_fMaxSteeringAngleRad) / angular_vel_steps;
    const float linear_vel_res = (fabs(m_pVehicleModel->m_fMaxReverseSpeed) + fabs(m_pVehicleModel->m_fMaxForwardSpeed)) / linear_vel_steps;

    const float linear_vel_start = -m_pVehicleModel->m_fMaxReverseSpeed;
    const float linear_vel_stop = m_pVehicleModel->m_fMaxForwardSpeed;
    const float angular_vel_start = -m_pVehicleModel->m_fMaxSteeringAngleRad;
    const float angular_vel_stop = m_pVehicleModel->m_fMaxSteeringAngleRad;

    const float goal_orientation = (goal_points.size() > 1U)
        ? get_angle(goal_points.front().r, goal_points.back().r)
        : 0.F;

    const float orientation_coeff = (goal_points.size() > 1U) ? m_config.orientation_coefficient : 0.F;

    //const auto segmentation_hull = CConcaveHull::calculate(traversable_nodes);

    int idx = 0;
    for (auto angular_vel = angular_vel_start; angular_vel < angular_vel_stop; angular_vel += angular_vel_res)
    {
        for (auto linear_vel = linear_vel_start; linear_vel < linear_vel_stop; linear_vel += linear_vel_res)
        {
            m_pVehicleModel->set_x(_vehicle_state.x_hat);
            m_pVehicleModel->set_y(m_pVehicleModel->x());

            CycSetPoints traj = generateTrajectory(linear_vel, angular_vel);
            if (_out_samples != nullptr)
                _out_samples->push_back(traj);

            const auto goal_cost = m_config.goal_coefficient * goalPointsCost(traj, goal_points); // Drive towards goal point(s)
            const auto obs_cost = obstacleCost(traj, obstacles); // No need for obstacle amplification; cost is big anyway
            const auto angular_momentum_cost = m_config.angular_momentum_coefficient * angularMomentumCost(angular_vel);  // Penalize oscillation
            const auto linear_momentum_cost = m_config.linear_momentum_coefficient * linearMomentumCost(linear_vel); // Penalize velocity differences
            const auto top_speed_cost = m_config.top_speed_coefficient * topSpeedCost(linear_vel); // Penalize low speed runs
            const auto lanes_cost = 0; // m_config.lanes_coefficient* lanesCost(traj, lanes);
            const auto orientation_cost = orientation_coeff * abs(goal_orientation - traj.back().r[3]);
            
            //const auto semseg_cost = m_config.semseg_coefficient * roadSegmentationCost(traj, traversable_nodes);
            const auto traversable_cost = 0.f; // m_config.traversable_coefficient* roadSegmentationHullCost(traj, segmentation_hull);
            
            const auto total_cost = goal_cost + obs_cost + angular_momentum_cost + linear_momentum_cost + top_speed_cost + lanes_cost + orientation_cost + traversable_cost;

            if (total_cost < min_cost && obs_cost == 0.f)
            {
                min_cost = total_cost;
                best_traj = traj;
                control.u << linear_vel, angular_vel;
                control.ref = best_traj;
            }

            ++idx;
        }
    }

    m_PreviousControl = control;

    //for (const auto& gp : goal_points)
    //    control.goal_points.push_back(gp);

    if (m_PreviousControlSignals.size() >= m_NumHistoryControlSignals)
        m_PreviousControlSignals.erase(m_PreviousControlSignals.begin());

    m_PreviousControlSignals.push_back(control);
    control = filterSignals(control);

    return control;
}

CycSetPoints CDwa::dwaPlan(const CycState& _vehicle_state, const CycEnvironment& _env, std::vector<CycSetPoints>* _out_samples)
{
    if (_out_samples != nullptr)
        _out_samples->clear();

    float min_cost = std::numeric_limits<float>::max();
    std::vector<CycSetPoint> best_traj;
    CycSetPoints reference;

    // TODO: remove hardcode
    float target_speed = 4.f;

    std::vector<CycSetPoint> obstacles, traversable;
    parse_octree(*_env.pOccupancyModel, obstacles, traversable);
    std::vector<CycSetPoint> goal_points;
    find_goal_points(_vehicle_state, goal_points);

    if (!m_GoalPointReached)
        if (goalPointReached(goal_points, _vehicle_state))
            return reference;

    const float goal_orientation = (goal_points.size() > 1U)
        ? get_angle(goal_points.front().r, goal_points.back().r)
        : 0.F;

    const float orientation_coeff = (goal_points.size() > 1U) ? m_config.orientation_coefficient : 0.F;

    for (float yaw = -m_pVehicleModel->m_fMaxSteeringAngleRad; yaw <= m_pVehicleModel->m_fMaxSteeringAngleRad; yaw += m_pVehicleModel->m_fSteeringRateResolution)
    {
        m_pVehicleModel->set_x(_vehicle_state.x_hat);
        m_pVehicleModel->set_y(m_pVehicleModel->x());

        std::vector<CycSetPoint> traj = generateTrajectory(target_speed, yaw);
        if (_out_samples != nullptr)
            _out_samples->push_back(traj);
        
        const float goal_cost = m_config.goal_coefficient * goalPointsCost(traj, goal_points); // Drive towards goal point(s)
        const float obs_cost = obstacleCost(traj, obstacles); // No need for obstacle amplification; cost is big anyway
        const float orientation_cost = orientation_coeff * abs(goal_orientation - traj.back().r[3]);

        const float traversable_cost = m_config.traversable_coefficient * traversableCost(traj, traversable);
        //spdlog::info("goal and traversable_costs = {},\t{}", goal_cost, traversable_cost);

        const float total_cost = traversable_cost + goal_cost + obs_cost + orientation_cost;

        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            best_traj = traj;
            reference = best_traj;
        }
    }

    //spdlog::info("TODO: ref: {}", reference.ref.size());
    //spdlog::info("---");

    return reference;
}
