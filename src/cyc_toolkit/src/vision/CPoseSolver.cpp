// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CPoseSolver.h"
#include <opencv2/core/eigen.hpp>

CPoseSolver::CPoseSolver()
{}

void CPoseSolver::getNearestT(const Eigen::Matrix4f& _T,
    const std::vector<Eigen::Matrix3f>& _Rs, 
    const std::vector<Eigen::Vector3f>& _ts, 
    Eigen::Matrix4f& _T_near,
    float& _out_rot_dist,
    float& _out_trans_dist)
{
    _out_rot_dist = 999999.f;
    _out_trans_dist = 999999.f;
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    
    // Get best rotation
    Eigen::Vector3f euler_given = CPose::R2euler(_T.block(0, 0, 3, 3));
    for (size_t i = 0; i < _Rs.size(); ++i)
    {
        Eigen::Vector3f euler = CPose::R2euler(_Rs[i]);
        float ed = CGeometry::euclidean_dist(euler_given, euler);

        if (ed < _out_rot_dist)
        {
            _out_rot_dist = ed;
            R = _Rs[i];
        }
    }

    // Get best translation
    Eigen::Vector3f trans_given = _T.block(0, 3, 3, 1);
    for (size_t i = 0; i < _ts.size(); ++i)
    {
        float ed = CGeometry::euclidean_dist(trans_given, _ts[i]);

        if (ed < _out_trans_dist)
        {
            _out_trans_dist = ed;
            t = _ts[i];
        }
    }
    
    _T_near = CPose::Rt2T(R, t);
}

void CPoseSolver::getNearestT(const Eigen::Matrix4f& _T,
    const std::vector<Eigen::Matrix3f>& _Es, 
    Eigen::Matrix4f& _T_near,
    float& _out_rot_dist,
    float& _out_trans_dist)
{
    _out_rot_dist = 999999.f;
    _out_trans_dist = 999999.f;
    
    // Parse the essential matrix solutions
    for (size_t i = 0; i < _Es.size(); ++i)
    {
        std::vector<Eigen::Matrix3f> T_Rots_from_E;
        std::vector<Eigen::Vector3f> T_Trans_from_E;
        CProjectiveGeometry::E2Rt(_Es[i], T_Rots_from_E, T_Trans_from_E);

        Eigen::Matrix4f T_from_E;
        float rot_dist, trans_dist;
        getNearestT(_T, T_Rots_from_E, T_Trans_from_E, T_from_E, rot_dist, trans_dist);

        if (rot_dist < _out_rot_dist)
        {
            _out_rot_dist = rot_dist;
            _T_near = T_from_E;
        }
    }
}

bool CPoseSolver::solveT(const CPinholeCameraSensorModel* _pSensorModel,
    const Eigen::Matrix3f& _E,
    const CycPoints& _pts1,
    const CycPoints& _pts2,
    const float _pos_depth_th,
    Eigen::Matrix4f& _out_T)
{
    std::vector<Eigen::Matrix4f> Ts_pos_depth;
    float min_angular_distance = 99999.9f;

    // Get projection matrices with positive depth
    std::vector<Eigen::Matrix4f> Ts;
    CProjectiveGeometry::E2T(_E, Ts);

    for (size_t i = 0; i < Ts.size(); ++i)
    {
        float depth_freq = CTriangulation::getPosDepthFreq(_pSensorModel, _pts1, _pts2, CPose().transform(), Ts[i]);
        if (depth_freq > _pos_depth_th)
            Ts_pos_depth.emplace_back(Ts[i]);
    }

    if (Ts_pos_depth.size() == 0)
        return false;

    // Parse the projection matrices candidates
    for (size_t i = 0; i < Ts_pos_depth.size(); ++i)
    {
        // Get nearest angular distance to the zero pose
        Eigen::Vector3f euler = CPose::R2euler(Ts_pos_depth[i].block(0, 0, 3, 3));
        float ang_dist = CGeometry::euclidean_dist(CPose().rotation_euler(), euler);

        if (ang_dist < min_angular_distance)
            _out_T = Ts_pos_depth[i];
    }

    return true;
}

bool CPoseSolver::solveT(const CPinholeCameraSensorModel* _pSensorModel,
    const std::vector<Eigen::Matrix3f>& _Es,
    const CycPoints& _pts1,
    const CycPoints& _pts2,
    const float _pos_depth_th,
    Eigen::Matrix4f& _out_T,
    Eigen::Matrix3f& _out_E)
{
    assert(_pts1.size() != _pts2.size());

    std::vector<Eigen::Matrix3f> Es_pos_depth;
    std::vector<Eigen::Matrix4f> Ts_pos_depth;
    float min_angular_distance = 99999.9f;
    
    // Get projection matrices with positive depth
    for (size_t k = 0; k < _Es.size(); ++k)
    {
        std::vector<Eigen::Matrix4f> Ts;
        CProjectiveGeometry::E2T(_Es[k], Ts);

        for (size_t i = 0; i < Ts.size(); ++i)
        {
            float depth_freq = CTriangulation::getPosDepthFreq(_pSensorModel, _pts1, _pts2, CPose().transform(), Ts[i]);
            if (depth_freq > _pos_depth_th)
            {
                Es_pos_depth.emplace_back(_Es[k]);
                Ts_pos_depth.emplace_back(Ts[i]);
            }
        }
    }

    if (Ts_pos_depth.size() == 0)
        return false;

    // Parse the projection matrices candidates
    for (size_t i = 0; i < Ts_pos_depth.size(); ++i)
    {
        // Get nearest angular distance to the zero pose
        Eigen::Vector3f euler = CPose::R2euler(Ts_pos_depth[i].block(0, 0, 3, 3));
        float ang_dist = CGeometry::euclidean_dist(CPose().rotation_euler(), euler);

        if (ang_dist < min_angular_distance)
        {
            _out_E = Es_pos_depth[i];
            _out_T = Ts_pos_depth[i];
        }
    }

    return true;
}

void CPoseSolver::cam_pose_candidates(const CPinholeCameraSensorModel* _pSensorModel,
    const std::vector<Eigen::Matrix3f>& _E_candidates,
    const CycPoints& _pts1,
    const CycPoints& _pts2,
    const float _min_pos_depth_freq,
    std::vector<Eigen::Matrix4f>& _out_pose_candidates)
{
    _out_pose_candidates.clear();

    // Get the candidates with a positive depth frequency higher than 0.9
    for (size_t i = 0; i < _E_candidates.size(); ++i)
    {
        std::vector<Eigen::Matrix4f> Ts;

        Eigen::Matrix3f E = _E_candidates[i];

        // Only one of the 4 transformations in T will give positive depth in both cameras
        CProjectiveGeometry::E2T(E, Ts);

        for (size_t j = 0; j < Ts.size(); ++j)
        {
            Eigen::Matrix4f T = Ts[j];

            // Triangulate
            CycVoxels voxels_triang;
            Eigen::Matrix4f cam_pose_2 = CProjectiveGeometry::invertT(T);
            CTriangulation::TriangulatePoints(_pSensorModel, _pts1, _pts2, CPose().transform(), cam_pose_2, voxels_triang);

            float pos_depth_freq = CTriangulation::getPosDepthFreq(_pSensorModel, CPose().transform(), cam_pose_2, voxels_triang);

            if (pos_depth_freq > _min_pos_depth_freq)
                _out_pose_candidates.emplace_back(cam_pose_2);
        }
    }
}
