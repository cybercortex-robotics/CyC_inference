// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CPoseSolver_H_
#define CPoseSolver_H_

#include "CCR_TYPES.h"
#include <cmath>
#include <math.h>
#include "CProjectiveGeometry.h"
#include "CTriangulation.h"
#include "math/CGeometry.h"
#include "sensors/CPinholeCameraSensorModel.h"

class CPoseSolver
{
public:
    CPoseSolver();
    CPoseSolver(const CPoseSolver&) = delete;
    CPoseSolver(CPoseSolver&&) = delete;
    CPoseSolver operator=(const CPoseSolver&) = delete;
    CPoseSolver operator=(CPoseSolver&&) = delete;
    ~CPoseSolver() = default;
 
    /**
     * \brief                   Computes the nearest transformation to a given pose from a vector of rotation and translation candidates
     *
     * \param _T                Input previous transformation
     * \param _Rs               Input vector of rotations
     * \param _ts               Input vector of translations
     * \param _T_near           Output transformation
     * \param _out_rot_dist     Output nearest rotation
     * \param _out_trans_dist   Output nearest translation
     **/
    static void getNearestT(const Eigen::Matrix4f& _T,
        const std::vector<Eigen::Matrix3f>& _Rs, 
        const std::vector<Eigen::Vector3f>& _ts, 
        Eigen::Matrix4f& _T_near,
        float& _out_rot_dist,
        float& _out_trans_dist);
    
    /**
     * \brief                   Computes the nearest transformation to a given pose from a vector of essential matrices candidates
     *
     * \param _T                Input previous transformation
     * \param _Es               Input vector of essential matrices
     * \param _T_near           Output transformation
     * \param _out_rot_dist     Output nearest rotation
     * \param _out_trans_dist   Output nearest translation
     **/
    static void getNearestT(const Eigen::Matrix4f& _T,
        const std::vector<Eigen::Matrix3f>& _Es, 
        Eigen::Matrix4f& _T_near,
        float& _out_rot_dist,
        float& _out_trans_dist);

    /**
     * \brief                   Solves transformation matrix computation by decomposing the essential matrix based on the reprojection error, positive depth frequency and angular distance
     *
     * \param _pSensorModel     Input sensor model
     * \param _E                Input essential matrix
     * \param _pts1             Input 2D point in the first image
     * \param _pts2             Input 2D point in the second image
     * \param _pos_depth_th     Input positive depth threshold
     * \param _out_T            Output transformation matrix
     * \return                  True if decomposition is solved
     **/
    static bool solveT(const CPinholeCameraSensorModel * _pSensorModel, 
        const Eigen::Matrix3f& _E,
        const CcrPoints&_pts1,
        const CcrPoints&_pts2,
        const float _pos_depth_th,
        Eigen::Matrix4f& _out_T);

    /**
     * \brief                   Solves transformation matrix computation by decomposing the essential matricx candidates based on the reprojection error, positive depth frequency and angular distance
     *
     * \param _pSensorModel     Input sensor model
     * \param _Es               Input vector of essential matrices
     * \param _pts1             Input 2D point in the first image
     * \param _pts2             Input 2D point in the second image
     * \param _pos_depth_th     Input positive depth threshold
     * \param _out_T            Output transformation matrix
     * \param _out_E            Output essential matrix from which _out_T was obtained
     * \return                  True if decomposition is solved
     **/
    static bool solveT(const CPinholeCameraSensorModel* _pSensorModel,
        const std::vector<Eigen::Matrix3f>& _Es,
        const CcrPoints& _pts1,
        const CcrPoints& _pts2,
        const float _pos_depth_th,
        Eigen::Matrix4f& _out_T,
        Eigen::Matrix3f& _out_E);

    static void cam_pose_candidates(const CPinholeCameraSensorModel* _pSensorModel,
        const std::vector<Eigen::Matrix3f>& _E_candidates,
        const CcrPoints& _pts1,
        const CcrPoints& _pts2,
        const float _min_pos_depth_freq,
        std::vector<Eigen::Matrix4f>& _out_pose_candidates);
};

#endif /* CPoseSolver_H_ */
