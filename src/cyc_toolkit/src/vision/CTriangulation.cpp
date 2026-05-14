// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CTriangulation.h"
#include <opencv2/core/eigen.hpp>

CTriangulation::CTriangulation()
{}

CycVoxel CTriangulation::TriangulatePoint(const CPinholeCameraSensorModel* _pSensorModel, const CycPoint& _pt1, const CycPoint& _pt2, const Eigen::Matrix4f& _cam_pose_1, const Eigen::Matrix4f& _cam_pose_2)
{
    // Undistort points
    const Eigen::Vector2f pt1_undist = CProjectiveGeometry::undistort(_pSensorModel, _pt1.pt2d);
    const Eigen::Vector2f pt2_undist = CProjectiveGeometry::undistort(_pSensorModel, _pt2.pt2d);

    Eigen::Matrix4f A;

    A(0, 0) = pt1_undist.x() * _cam_pose_1(2, 0) - _cam_pose_1(0, 0);
    A(0, 1) = pt1_undist.x() * _cam_pose_1(2, 1) - _cam_pose_1(0, 1);
    A(0, 2) = pt1_undist.x() * _cam_pose_1(2, 2) - _cam_pose_1(0, 2);
    A(0, 3) = pt1_undist.x() * _cam_pose_1(2, 3) - _cam_pose_1(0, 3);

    A(1, 0) = pt1_undist.y() * _cam_pose_1(2, 0) - _cam_pose_1(1, 0);
    A(1, 1) = pt1_undist.y() * _cam_pose_1(2, 1) - _cam_pose_1(1, 1);
    A(1, 2) = pt1_undist.y() * _cam_pose_1(2, 2) - _cam_pose_1(1, 2);
    A(1, 3) = pt1_undist.y() * _cam_pose_1(2, 3) - _cam_pose_1(1, 3);
     
    A(2, 0) = pt2_undist.x() * _cam_pose_2(2, 0) - _cam_pose_2(0, 0);
    A(2, 1) = pt2_undist.x() * _cam_pose_2(2, 1) - _cam_pose_2(0, 1);
    A(2, 2) = pt2_undist.x() * _cam_pose_2(2, 2) - _cam_pose_2(0, 2);
    A(2, 3) = pt2_undist.x() * _cam_pose_2(2, 3) - _cam_pose_2(0, 3);

    A(3, 0) = pt2_undist.y() * _cam_pose_2(2, 0) - _cam_pose_2(1, 0);
    A(3, 1) = pt2_undist.y() * _cam_pose_2(2, 1) - _cam_pose_2(1, 1);
    A(3, 2) = pt2_undist.y() * _cam_pose_2(2, 2) - _cam_pose_2(1, 2);
    A(3, 3) = pt2_undist.y() * _cam_pose_2(2, 3) - _cam_pose_2(1, 3);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

    return CycVoxel { svd.matrixV().col(3), _pt1.id };
}

CycVoxel CTriangulation::Triangulate(const CycPoint& _pt1,
    const CycPoint& _pt2,
    const Pmatrix& _P1,
    const Pmatrix& _P2)
{
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
    A.row(0) = _pt1.pt2d.x() * _P1.row(2) - _P1.row(0);
    A.row(1) = _pt1.pt2d.y() * _P1.row(2) - _P1.row(1);
    A.row(2) = _pt2.pt2d.x() * _P2.row(2) - _P2.row(0);
    A.row(3) = _pt2.pt2d.y() * _P2.row(2) - _P2.row(1);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

    CycVoxel vx(svd.matrixV().col(3) / svd.matrixV().col(3)(3), _pt1.id);

    return vx;
}

CycVoxel CTriangulation::Triangulate(const CPinholeCameraSensorModel* _pSensorModel,
    const CycPoint& _pt1,
    const CycPoint& _pt2,
    const Eigen::Matrix4f& _cam_pose_1,
    const Eigen::Matrix4f& _cam_pose_2)
{
    Pmatrix P1;
    Pmatrix P2;

    P1 = _pSensorModel->K() * CProjectiveGeometry::invertT(_cam_pose_1).block(0, 0, 3, 4);
    P2 = _pSensorModel->K() * CProjectiveGeometry::invertT(_cam_pose_2).block(0, 0, 3, 4);

    return Triangulate(_pt1, _pt2, P1, P2);
}

void CTriangulation::TriangulatePoints(const CycPoints& _pts1,
    const CycPoints& _pts2,
    const Pmatrix& _P1,
    const Pmatrix& _P2,
    CycVoxels& _out_pts3d)
{
    assert(_pts1.size() != _pts2.size());

    _out_pts3d.clear();

    for (size_t i = 0; i < _pts1.size(); ++i)
        _out_pts3d.emplace_back(Triangulate(_pts1[i], _pts2[i], _P1, _P2));
}
/*
void CTriangulation::TriangulatePoints(const CycPoints& _pts1,
    const CycPoints& _pts2,
    const Pmatrix& _P1,
    const Pmatrix& _P2,
    CycVoxels& _out_pts3d)
{
    assert(_pts1.size() != _pts2.size());

    _out_pts3d.clear();

    for (size_t i = 0; i < _pts1.size(); ++i)
    {
        Eigen::Vector4f pt3d = Triangulate(_pts1[i].pt2d, _pts2[i].pt2d, _P1, _P2);
        CycVoxel vx(pt3d, _pts1[i].id, _pts1[i].score);
        _out_pts3d.emplace_back(vx);
    }
}
*/
void CTriangulation::TriangulatePoints(const CPinholeCameraSensorModel* _pSensorModel,
    const CycPoints& _pts1,
    const CycPoints& _pts2,
    const Eigen::Matrix4f& _cam_pose_1,
    const Eigen::Matrix4f& _cam_pose_2,
    CycVoxels& _out_pts3d)
{
    assert(_pts1.size() != _pts2.size());

    _out_pts3d.clear();

    Pmatrix P1;
    Pmatrix P2;

    // Invert camera pose in order to calculate the projection matrices
    P1 = _pSensorModel->K() * CProjectiveGeometry::invertT(_cam_pose_1).block(0, 0, 3, 4);
    P2 = _pSensorModel->K() * CProjectiveGeometry::invertT(_cam_pose_2).block(0, 0, 3, 4);

    TriangulatePoints(_pts1, _pts2, P1, P2, _out_pts3d);
}

void CTriangulation::TriangulatePoints(const CPinholeCameraSensorModel* _pSensorModel,
    const CycPoints& _pts1,
    const CycPoints& _pts2,
    const CPose& _cam_pose_1,
    const CPose& _cam_pose_2,
    CycVoxels& _out_pts3d)
{
    TriangulatePoints(_pSensorModel,
        _pts1,
        _pts2,
        _cam_pose_1.transform(),
        _cam_pose_2.transform(),
        _out_pts3d);
}

float CTriangulation::getDepth(const CycVoxel& _voxel, const Pmatrix& _P)
{
    // back project
    // _X is 4x1 is [x,y,z,w]
    // _P is 3x4 projection matrix
    Eigen::Vector3f X2 = _P * _voxel.pt3d;

    float det = _P.block(0, 0, 3, 3).determinant();
    float w = X2(2, 0);
    float W = _voxel.pt3d(3, 0);

    float a = _P(0, 2);
    float b = _P(1, 2);
    float c = _P(2, 2);

    float m3 = sqrt(a * a + b * b + c * c);  // 3rd column of M

    float sign;

    if (det > 0.f)
        sign = 1.f;
    else
        sign = -1.f;
    
    return (w / W) * (sign / m3);
}

float CTriangulation::getPosDepthFreq(const CPinholeCameraSensorModel* _pSensorModel,
    const Eigen::Matrix4f& _cam_pose_1,
    const Eigen::Matrix4f& _cam_pose_2,
    const CycVoxels& _voxels)
{
    // Calculate projection matrices
    Pmatrix P_1 = CProjectiveGeometry::KT2P(_pSensorModel->K(), CProjectiveGeometry::invertT(_cam_pose_1));
    Pmatrix P_2 = CProjectiveGeometry::KT2P(_pSensorModel->K(), CProjectiveGeometry::invertT(_cam_pose_2));

    CyC_INT pos_depth_counter = 0;
    for (size_t k = 0; k < _voxels.size(); ++k)
    {
        float depth1 = CTriangulation::getDepth(_voxels[k], P_1);
        float depth2 = CTriangulation::getDepth(_voxels[k], P_2);

        if (depth1 > 0.f && depth2 > 0.f)
            pos_depth_counter++;
    }

    return (float)pos_depth_counter / (float)_voxels.size();
}




float CTriangulation::getPosDepthFreq(const CPinholeCameraSensorModel* _pSensorModel,
    const CycPoints& _pts1,
    const CycPoints& _pts2,
    const Eigen::Matrix4f& _P1,
    const Eigen::Matrix4f& _P2)
{
    assert(_pts1.size() != _pts2.size());
    
    CycVoxels voxels;
    for (size_t j = 0; j < _pts1.size(); ++j)
    {
        CycVoxel pt3d_triang = TriangulatePoint(_pSensorModel, _pts1[j], _pts2[j], _P1, _P2);
        voxels.emplace_back(pt3d_triang);
    }
    
    return 0; //getPosDepthFreq(_pSensorModel, voxels, _P1, _P2);
}

float CTriangulation::getPosDepthFreq(const CPinholeCameraSensorModel* _pSensorModel,
    const CycVoxels& _voxels,
    const Eigen::Matrix4f& _P1,
    const Eigen::Matrix4f& _P2)
{
    int num_pos_depth_pts = 0;
    /*
    for (size_t i = 0; i < _voxels.size(); ++i)
    {
        float depth1 = getDepth(_voxels[i], _P1);
        float depth2 = getDepth(_voxels[i], _P2);

        if (depth1 > 0.f && depth2 > 0.f)
            num_pos_depth_pts++;
    }
    */
    return (float)num_pos_depth_pts / (float)_voxels.size();
}
