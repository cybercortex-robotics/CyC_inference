// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CDwa_H_
#define CDwa_H_

#include <Eigen/Eigen>
#include <Eigen/Core>
#include "CyC_TYPES.h"
#include "control/CModelVehicle.h"
#include "plan/CPlanningUtils.h"

class CDwa
{
public:
    CDwa(const float _dt,
        const std::string& _vehicle_model_file,
        const float _lookahead_distance, 
        const float& _goal_distance,
        const std::vector<CyC_INT>& _travesable_class_ids = std::vector<CyC_INT>());

    bool isEnabled() { return m_bEnabled; };

    void setMissionPath(const std::vector<Eigen::Vector4f>& _mission_path, const CycState& _vehicle_state);

    CycControlInput dwaControl(
        const CycState& vehicle_state,
        const CycEnvironment& env);

    CycReferenceSetPoints dwaPlan(
        const CycState& vehicle_state,
        const CycEnvironment& env);

private:
    void find_goal_points(const CycState& _vehicle_state, std::vector<Eigen::VectorXf>& _out_goal_points);
    void parse_octree(const CcrOcTree& _octree, std::vector<Eigen::VectorXf>& _out_obstacles, std::vector<Eigen::VectorXf>& _out_traversable);
    std::vector<Eigen::VectorXf> find_traversable_nodes(const CcrOcTree& _octree, const CyC_INT& _traversable_class_id);
    bool goalPointReached(const std::vector<Eigen::VectorXf>& goal_points, const CycState& _vehicle_state);
    CycControlInput filterSignals(const CycControlInput& raw_control);

    // Use the underlying vehicle model to predict the vehicle state over the dynamic window
    void predictState(float vel, float steer);

    // Check if the state measurement 'state' collides with the obstacle point
    bool obstacleCollision(const Eigen::VectorXf& state, const Eigen::VectorXf& obstacle_point);

    // Cost for (not) hitting goal point(s)
    float goalPointsCost(const std::vector<Eigen::VectorXf>& trajectory, const std::vector<Eigen::VectorXf>& goal_points);

    // Penalty for differences between successive angular velocity commands (oscillation)
    float angularMomentumCost(float angular_vel);

    // Penalty for differences between successive linear velocity commands
    float linearMomentumCost(float linear_vel);

    // Penalty for going below top speed
    float topSpeedCost(float linear_vel);

    // Hitting an obstacle - should be very large when hitting an obstacle; zero otherwise
    float obstacleCost(const std::vector<Eigen::VectorXf>& trajectory, const std::vector<Eigen::VectorXf>& obstacles);

    // Traversable cost - is inverse proportional to the number of traversable nodes in the environment model
    float traversableCost(const std::vector<Eigen::VectorXf>& trajectory, const std::vector<Eigen::VectorXf>& traversable);

    // Cost for staying in between lanes
    float lanesCost(const std::vector<Eigen::VectorXf>& trajectory, const CycLanesModel& lanes);

    // Cost for staying on the segmented road
    //float roadSegmentationCost(const std::vector<Eigen::VectorXf>& trajectory, const CycImages& semseg_imgs);
    float roadSegmentationCost(const CycTrajectory& trajectory, const std::vector<Eigen::VectorXf>& traversable_nodes);
    float roadSegmentationHullCost(const CycTrajectory& trajectory, const std::vector<Eigen::Vector4f>& hull_nodes);

    // Generate trajectory with given linear velocity and agular velocity commands
    std::vector<Eigen::VectorXf> generateTrajectory(float target_speed, float steer);

    // Euclidean distance
    float distance(const Eigen::VectorXf& pt1, const Eigen::VectorXf& pt2)
    {
        return sqrt(powf(pt2(0) - pt1(0), 2) + powf(pt2(1) - pt1(1), 2));
    }

private:
    float m_bEnabled = false;

    struct DWAConfig
    {
        float dt;
        float angular_momentum_coefficient;
        float linear_momentum_coefficient;
        float top_speed_coefficient;
        float goal_coefficient;
        float lanes_coefficient;
        float lookahead_distance;
        float orientation_coefficient;
        float traversable_coefficient;
        
        DWAConfig() :
            dt(0.1f),
            angular_momentum_coefficient(1.f),
            linear_momentum_coefficient(1.f),
            top_speed_coefficient(1.f),
            goal_coefficient(5.f),
            lanes_coefficient(5.f),
            lookahead_distance(2.f),
            orientation_coefficient(1.f),
            traversable_coefficient(1000.f)
        {}
    };

    std::unique_ptr<CModelVehicle>  m_pVehicleModel; // Vehicle state space model
    DWAConfig                       m_config;
    std::vector<Eigen::VectorXf>    m_previous_trajectory;

    std::vector<Eigen::Vector4f>    m_MissionPath;
    CyC_UINT                        m_NumGoalPoints = 1;
    float                           m_GoalDistance = 1.f;
    size_t                          m_PreviousTrajectoryPointIndex = 0;
    CyC_UINT                        m_ClosestIndex = 0;
    bool                            m_GoalPointReached = false;

    std::vector<CycControlInput>    m_PreviousControlSignals;
    std::vector<Eigen::VectorXf>    m_HistoryOfControlSignals;
    CyC_UINT                        m_NumHistoryControlSignals = 10;

    // Environment variables
    std::vector<CyC_INT>            m_TravesableClassIDs;

    // Previous control input (used for successive commands cost functions)
    CycControlInput                 m_PreviousControl;
};

#endif //CDwa_H_