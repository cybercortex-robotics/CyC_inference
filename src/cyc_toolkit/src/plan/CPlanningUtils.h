// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CPLANNINGUTILS_H_
#define CPLANNINGUTILS_H_

#include "CyC_TYPES.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "vision/CImageDisplayUtils.h"

class CPlanningUtils
{
public:
    CPlanningUtils();
    virtual ~CPlanningUtils();

    /**
    * \brief Display the AStar map of obstacles and generated path in the A* coordinates system
    *
    * \param _disp_img          Output display image
    * \param _local_ref_path    Point coordinates from the generated path
    * \param _fake__obstacles   Input fake obstacles point coordinates
    * \param _resolution        Input gridmap resolution (equal to the original octree resolution)
    * \param _angle             Vehicle yaw angle
    **/
    static void plotReferenceSetpointsOnGridmap(cv::Mat& _disp_img, const CycReferenceSetPoints& _local_ref_path, const Eigen::MatrixXi& _gridmap, const float _angle = 0.F, const std::vector<Eigen::Vector2f>& _fake__obstacles = std::vector<Eigen::Vector2f>());

    /**
    * \brief Display the global mission planner's path in the A* coordinate system
    *
    * \param _disp_img          Output display image
    * \param _global_path       Point coordinates from the generated path
    * \param _vehicle_state     State of the vehicle (x, y, yaw)
    * \param _octree_resolution Input octree resolution (in meters)
    * \param _gridmap_size      Input gridmap size (in meters)
    **/
    static void plotPath(cv::Mat& _disp_img, const std::vector<Eigen::Vector4f>& _global_path, const CycState& _vehicle_state, const Eigen::MatrixXi& _gridmap, const float& _octree_resolution, const cv::Scalar& _color);

    /**
    * \brief Display the global mission planner's path in the A* coordinate system
    *
    * \param _disp_img          Output display image
    * \param _ref_setpoints     Point coordinates from the Astar path
    * \param _vehicle_state     State of the vehicle (x, y, v, yaw)
    * \param _octree_resolution Input octree resolution (in meters)
    * \param _gridmap_size      Input gridmap size (in meters)
    **/
    static void plotReferenceSetpoints(cv::Mat& _disp_img, const std::vector<Eigen::VectorXf>& _ref_setpoints, const CycState& _vehicle_state, const Eigen::MatrixXi& _gridmap, const float& _octree_resolution);

    /**
    * \brief Display the goal point of A* the A* coordinate system
    *
    * \param _disp_img          Output display image
    * \param _goal_point        Point coordinates of the destination point
    * \param _vehicle_state     State of the vehicle (x, y, v, yaw)
    * \param _octree_resolution Input octree resolution (in meters)
    * \param _gridmap_size      Input gridmap size (in meters)
    **/
    static void plotAStarGoalPoint(cv::Mat& _disp_img, const Eigen::Vector4f &_goal_point, const CycState &_vehicle_state, const Eigen::MatrixXi& _gridmap, const float& _octree_resolution);

    /**
    * \brief Plot candidate trajectories e.g. DWA can produce multiple outputs
    * \param _disp_img                       Image where to plot
    * \param _candidate_trajectories          trajectories to plot (vector of vectors of Eigen Vectors)
    * \param _vehicle_state                  State of the vehicle (x, y, v, yaw)
    * \param _gridmap                        Input gridmap
    * \param _octree_resolution              Resolution of the octree
    **/
    static void plotCandidateTrajectories(cv::Mat & _disp_img, const std::vector<std::vector<Eigen::VectorXf>>& _candidate_trajectories, const CycState & _vehicle_state, const Eigen::MatrixXi & _gridmap, const float & _octree_resolution);

    /**
    * \brief Find closest point from trajectory to the vehicle current position
    *
    * \param _mission_path                    Trajectory vector
    * \param _vehicle_state                   Current vehicle state
    * \param _previous_trajectory_point_index Previously selected trajectory point index
    **/
    static size_t findClosestPoint(const std::vector<Eigen::Vector4f>& _mission_path, const CycState& _vehicle_state, size_t _previous_trajectory_point_index, size_t _closest_index=0);
};

#endif /* CPLANNINGUTILS_H_ */
