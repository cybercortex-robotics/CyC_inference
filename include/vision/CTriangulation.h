// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/*
 * cheirality condition: the reconstructed points must be in front of the cameras. 
 * To check the cheirality condition, triangulate the 3D points (given two camera poses) using linear least squares to check the sign of the 
 * depth Z in the camera coordinate system w.r.t. camera center. A 3D point X is in front of the camera iff: r3(X?C)>0 where r3 is the third row 
 * of the rotation matrix (z-axis of the camera).
 * 
 * https://imkaywu.github.io/blog/2017/07/triangulation/
 * http://nghiaho.com/?p=2379; https://github.com/nghiaho12/SFM_example
 * http://cvg.ethz.ch/mobile/LiveMetric3DReconstructionICCV2013.pdf
 * http://phototour.cs.washington.edu/ModelingTheWorld_ijcv07.pdf
 * https://www.uio.no/studier/emner/matnat/its/nedlagte-emner/UNIK4690/v16/forelesninger/lecture_7_2-triangulation.pdf
 * https://github.com/ethz-asl/maplab/blob/master/algorithms/geometric-vision-algorithms/src/linear-triangulation.cc
 * https://github.com/PavelNajman/Triangulation/blob/master/LinearLS.cpp
 */
 
#ifndef CTriangulation_H_
#define CTriangulation_H_

#include "CyC_TYPES.h"
#include <cmath>
#include <math.h>
#include "CProjectiveGeometry.h"
#include "sensors/CPinholeCameraSensorModel.h"

class CTriangulation
{
public:
    CTriangulation();
    CTriangulation(const CTriangulation&) = delete;
    CTriangulation(CTriangulation&&) = delete;
    CTriangulation operator=(const CTriangulation&) = delete;
    CTriangulation operator=(CTriangulation&&) = delete;
    ~CTriangulation() = default;

    /**
     * \brief Triangulate point by undistorting the corresponding points using the camera matrix
     *
     * \param _pSensorModel     Pinhole camera sensor model
     * \param _pt1              Correspondence point from the first image
     * \param _pt2              Correspondence point from the second image
     * \param _cam_pose_1       First camera pose matrix
     * \param _cam_pose_2       Second camera pose matrix
     * \return                  Triangulated 3D point
     **/
    static CycVoxel TriangulatePoint(const CPinholeCameraSensorModel* _pSensorModel, 
        const CycPoint& _pt1, 
        const CycPoint& _pt2,
        const Eigen::Matrix4f& _cam_pose_1, 
        const Eigen::Matrix4f& _cam_pose_2);

    /**
     * \brief Triangulate point using projection matrices
     *
     * \param _pt1              Correspondence point from the first image
     * \param _pt2              Correspondence point from the second image
     * \param _P1               3x4 projection matrix of camera 1, composed of intrinsics * camera_pose_1
     * \param _P2               3x4 projection matrix of camera 2, composed of intrinsics * camera_pose_2
     * \return                  Triangulated 3D point
     **/
    static CycVoxel Triangulate(const CycPoint& _pt1,
        const CycPoint& _pt2,
        const Pmatrix& _P1,
        const Pmatrix& _P2);

    /**
     * \brief Triangulate point using projection matrices
     *
     * \param _pSensorModel     Pinhole camera sensor model
     * \param _pt1              Correspondence point from the first image
     * \param _pt2              Correspondence point from the second image
     * \param _cam_pose_1       First camera pose matrix
     * \param _cam_pose_2       Second camera pose matrix
     * \return                  Triangulated 3D point
     **/
    static CycVoxel Triangulate(const CPinholeCameraSensorModel* _pSensorModel,
        const CycPoint& _pt1,
        const CycPoint& _pt2,
        const Eigen::Matrix4f& _cam_pose_1,
        const Eigen::Matrix4f& _cam_pose_2);

    /**
    * \brief Triangulate points using projection matrices
    *
    * \param _pts1              Correspondence points from the first image
    * \param _pts2              Correspondence points from the second image
    * \param _P1                3x4 projection matrix of camera 1, composed of intrinsics * camera_pose_1
    * \param _P2                3x4 projection matrix of camera 2, composed of intrinsics * camera_pose_2
    * \return _out_pts3d        Vector of triangulated 3D points
    **/
    static void TriangulatePoints(const CycPoints& _pts1,
        const CycPoints& _pts2,
        const Pmatrix& _P1,
        const Pmatrix& _P2,
        CycVoxels& _out_pts3d);

    /**
   * \brief Triangulate points using projection matrices
   *
   * \param _pts1              Correspondence points from the first image
   * \param _pts2              Correspondence points from the second image
   * \param _P1                3x4 projection matrix of camera 1, composed of intrinsics * camera_pose_1
   * \param _P2                3x4 projection matrix of camera 2, composed of intrinsics * camera_pose_2
   * \return _out_pts3d        Vector of triangulated 3D points
   **/
    /*static void TriangulatePoints(const CycPoints& _pts1,
        const CycPoints& _pts2,
        const Pmatrix& _P1,
        const Pmatrix& _P2,
        CycVoxels& _out_pts3d);*/

    /**
    * \brief Triangulate points using projection matrices
    *
    * \param _pSensorModel      Pinhole camera sensor model
    * \param _pts1              Correspondence points from the first image
    * \param _pts2              Correspondence points from the second image
    * \param _cam_pose_1        First camera pose matrix
    * \param _cam_pose_2        Second camera pose matrix
    * \return _out_pts3d        Vector of triangulated 3D points
    **/
    static void TriangulatePoints(const CPinholeCameraSensorModel* _pSensorModel,
        const CycPoints& _pts1,
        const CycPoints& _pts2,
        const Eigen::Matrix4f& _cam_pose_1,
        const Eigen::Matrix4f& _cam_pose_2,
        CycVoxels& _out_pts3d);

    /**
    * \brief Triangulate points using projection matrices
    *
    * \param _pSensorModel      Pinhole camera sensor model
    * \param _pts1              Correspondence points from the first image
    * \param _pts2              Correspondence points from the second image
    * \param _cam_pose_1        First camera pose
    * \param _cam_pose_2        Second camera pose
    * \return _out_pts3d        Vector of triangulated 3D points
    **/
    static void TriangulatePoints(const CPinholeCameraSensorModel* _pSensorModel,
        const CycPoints& _pts1,
        const CycPoints& _pts2,
        const CPose& _cam_pose_1,
        const CPose& _cam_pose_2,
        CycVoxels& _out_pts3d);

    // TODO: check
    static float getDepth(const CycVoxel& _voxel, const Pmatrix& _P);
    static float getPosDepthFreq(const CPinholeCameraSensorModel* _pSensorModel,
        const Eigen::Matrix4f& _cam_pose_1,
        const Eigen::Matrix4f& _cam_pose_2,
        const CycVoxels& _voxels);
    // !TODO: check

    // Calculates the positive depth frequency
    static float getPosDepthFreq(const CPinholeCameraSensorModel* _pSensorModel,
        const CycPoints& _pts1,
        const CycPoints& _pts2,
        const Eigen::Matrix4f& _P1,
        const Eigen::Matrix4f& _P2);
        
    static float getPosDepthFreq(const CPinholeCameraSensorModel* _pSensorModel,
        const CycVoxels& _voxels,
        const Eigen::Matrix4f& _P1,
        const Eigen::Matrix4f& _P2);
};

#endif /* CTriangulation_H_ */
