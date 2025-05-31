// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/*
 * R - 3x3 rotation matrix
 * t - 3x1 translation vector
 * T - 4x4 [R|t] transformation matrix
 * K - 3x3 intrinsic camera matrix
 * P - 3x4 camera projection matrix
 * E - essential matrix
 * T from E - describes how the world is transformed relative to the camera; projects 3D points onto a 2D image; is the inverse of pose
 * cam pose - 4x4 T matrix (describes the camera position and orientation w.r.t. the world; is used for triangulation; is the inverse of Ext)
 * 
 * 
 * References:
 * 
 * Bundle SFM: https://github.com/snavely/bundler_sfm
 * 
 * https://nghiaho.com/?p=1675
 * https://www.robots.ox.ac.uk/~az/tutorials/tutoriala.pdf
 * https://math.stackexchange.com/questions/455128/how-to-find-an-all-in-one-2d-to-3d-transformation-matrix-for-perspective-project
 * https://stackoverflow.com/questions/64260851/cv2-triangulatepoints-always-returns-same-z-value
 * https://stackoverflow.com/questions/69771738/svd-not-giving-rotation-and-translation-matrix-from-stereo-essential-matrix
 * https://github.com/laavanyebahl/3D-Reconstruction-and-Epipolar-Geometry
 * https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=8942150&fileOId=8942168
 * https://www.diva-portal.org/smash/get/diva2:1220622/FULLTEXT01.pdf
 * https://stackoverflow.com/questions/50188473/why-translation-is-the-null-vector-of-essential-matrix
 * https://medium.com/nerd-for-tech/visual-perception-epipolar-geometry-bcd63e7a8faa
 * http://www.diegm.uniud.it/fusiello/teaching/mvg/icvss13.pdf
 * https://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/
 * https://gist.github.com/royshil/7087bc2560c581d443bc#file-simpleadhoctracker-cpp-L101
 * 
 * https://stackoverflow.com/questions/18018924/projection-matrix-from-fundamental-matrix
 * https://medium.com/nerd-for-tech/visual-perception-epipolar-geometry-bcd63e7a8faa
 * http://www.cs.cornell.edu/~snavely/bundler/bundler-v0.4-manual.html
 * https://math.stackexchange.com/questions/82602/how-to-find-camera-position-and-rotation-from-a-4x4-matrix
 * https://www.maths.lth.se/matematiklth/personal/calle/datorseende13/notes/forelas6.pdf
 * https://stackoverflow.com/questions/16639106/camera-motion-from-corresponding-images
 * https://ksimek.github.io/2012/08/22/extrinsic/
 * 
 * https://stackoverflow.com/questions/17929667/epipolar-lines-with-known-rotation-and-translation
 * https://cs.stackexchange.com/questions/41327/what-is-the-difference-between-the-fundamental-matrix-and-the-essential-matrix
 * https://www.cse.unr.edu/~bebis/CS791E/Notes/EpipolarGeonetry.pdf
 * http://www.cse.psu.edu/~rtc12/CSE486/lecture19.pdf
 * https://stackoverflow.com/questions/41668649/how-is-a-line-represented-in-epipolar-geometry
 */
 
#ifndef CProjectiveGeometry_H_
#define CProjectiveGeometry_H_

#include "CyC_TYPES.h"
#include <cmath>
#include <math.h>
#include "math/CGeometry.h"
#include "sensors/CPinholeCameraSensorModel.h"

// 3x4 projection matrix
typedef Eigen::Matrix<float, 3, 4> Pmatrix;

namespace CProjectiveGeometry
{
    /**
     * \brief       Transforms a vector to its skew-symmetric matrix representation
     *
     * \param _t    Input translation vector
     * \return      3x3 skew symmetric matrix
     **/
    Eigen::Matrix3f vector2skewsymmetric(const Eigen::Vector3f& _t);

    /**
     * \brief           Adds a translation scale factor to a transformation matrix
     *
     * \param _t        Input T matrix
     * \param _scale    Input scale factor
     * \return          Scaled transformation matrix T
     **/
    Eigen::Matrix4f addScaleFactor(const Eigen::Matrix4f& _T, const float& _scale);

    /**
     * \brief                   Calculates a scale factor based on two known coordinates
     *
     * \param _trans_first_1    Input real world transform
     * \param _trans_first_2    Input real world transform
     * \param _trans_second_1   Input estimated transform
     * \param _trans_second_2   Input estimated transform
     * \return                  Scale factor
     **/
    float getScaleFactor(const Eigen::Vector4f& _trans_first_1, const Eigen::Vector4f& _trans_first_2,
        const Eigen::Vector4f& _trans_second_1, const Eigen::Vector4f& _trans_second_2);

    /**
     * \brief                   Calculates a scale factor based on real-world voxels _vxs_real and and their corresponding estimated voxels _vxs_estimated
     *
     * \param _vxs_real         Input real world voxels
     * \param _vxs_estimated    Input corresponding estimated voxels
     * \return                  Scale factor
     **/
    float getScaleFactor(const CycVoxels& _vxs_real, const CycVoxels& _vxs_estimated);

    /**
     * \brief           Calculates the camera projection matrix based on the camera intrinsic matrix and the camera's extrinsics
     *
     * \param _K        Input intrinsic matrix
     * \param _T        Input transformation matrix
     * \return          Projection matrix
     **/
    Pmatrix KT2P(const Eigen::Matrix3f& _K, const Eigen::Matrix4f& _T);

    /**
     * \brief           Projects a 3D point (voxel) to a 2D image using a 3x4 camera projection matrix
     *
     * \param _P        Input projection matrix
     * \param _voxel    Input voxel (3D point)
     * \return          Projected 2D point in image coordinates
     **/
    CycPoint project(const Pmatrix & _P, const CycVoxel& _voxel);

    /**
     * \brief               Projects a 3D point (voxel) to a 2D image using a 3x4 camera projection matrix
     *
     * \param _pSensorModel Input sensor model
     * \param _cam_pose     Input camera pose
     * \return              Projected 2D point in image coordinates
     **/
    CycPoint project(const CPinholeCameraSensorModel* _pSensorModel, const CPose& _cam_pose, const CycVoxel& _voxel);
    CycPoint project(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Matrix4f& _cam_pose, const CycVoxel& _voxel);

    /**
     * \brief               Projects a vector of 3D points (voxels) to a 2D image using the sensor model and camera pose
     *
     * \param _pSensorModel Input sensor model
     * \param _cam_pose     Input camera pose
     * \param _voxels       Input 3D points (voxels)
     * \param _out_pts      Output projected 2D points in image coordinates
     **/
    void project(const CPinholeCameraSensorModel* _pSensorModel,
        const Eigen::Matrix4f& _cam_pose,
        const CycVoxels& _voxels,
        CycPoints& _out_pts);

    /**
     * \brief               Projects a vector of 3D points (voxels) to a 2D image using the sensor model and camera pose
     *
     * \param _pSensorModel Input sensor model
     * \param _cam_pose     Input camera pose
     * \param _voxels       Input 3D points (voxels)
     * \param _out_pts      Output projected 2D points in image coordinates
     **/
    void project(const CPinholeCameraSensorModel* _pSensorModel,
        const CPose& _cam_pose,
        const CycVoxels& _voxels,
        CycPoints& _out_pts);

    /**
     * \brief       Inverts a transformation matrix T
     *
     * \param _T    Input transformation matrix
     * \return      Inverted transformation matrix
     **/
    Eigen::Matrix4f invertT(const Eigen::Matrix4f& _T);

    /**
     * \brief           Calculates the essential matrix from a rotation matrix and a translation vector
     *
     * \param _R        Input rotation matrix
     * \param _t        Input translation vector
     * \return          Essential matrix
     **/
    Eigen::Matrix3f Rt2E(const Eigen::Matrix3f& _R, const Eigen::Vector3f& _t);

    /**
     * \brief           Calculates the essential matrix from a transformation matrix
     *
     * \param _T        Input transformation matrix
     * \return          Essential matrix
     **/
    Eigen::Matrix3f T2E(const Eigen::Matrix4f& _T);

    /**
     * \brief           Computes four transformation solutions from an essential matrix
     *
     * \param _E        Input essential matrix
     * \param _T1       Output solution candidate 1
     * \param _T2       Output solution candidate 2
     * \param _T3       Output solution candidate 3
     * \param _T4       Output solution candidate 4
     **/
    void E2T(const Eigen::Matrix3f& _E, Eigen::Matrix4f& _T1, Eigen::Matrix4f& _T2, Eigen::Matrix4f& _T3, Eigen::Matrix4f& _T4);
    
    /**
     * \brief           Computes four transformation solutions from an essential matrix
     *
     * \param _E        Input essential matrix
     * \param _Ts       Output vector of solution candidates
     **/
    void E2T(const Eigen::Matrix3f& _E, std::vector<Eigen::Matrix4f>& _Ts);
    
    /**
     * \brief           Computes four rotation and four translation solutions from an essential matrix
     *
     * \param _E        Input essential matrix
     * \param _Rs       Output vector of rotation solutions
     * \param _ts       Output vector of translation solutions
     **/
    void E2Rt(const Eigen::Matrix3f& _E, std::vector<Eigen::Matrix3f>& _Rs, std::vector<Eigen::Vector3f>& _ts);

    /**
     * \brief           Computes the fundamental matrix from the essential matrix
     *
     * \param _K1       Input left intrinsic matrix
     * \param _K2       Input right intrinsic matrix
     * \param _E        Input essential matrix
     * \return          Output fundamental matrix
     **/
    Eigen::Matrix3f E2F(const Eigen::Matrix3f& _K1, const Eigen::Matrix3f& _K2, const Eigen::Matrix3f& _E);

    /**
     * \brief               Computes the fundamental matrix from the essential matrix
     *
     * \param _pSensorModel Input sensor model
     * \param _E            Input essential matrix
     * \return              Output fundamental matrix
     **/
    Eigen::Matrix3f E2F(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Matrix3f& _E);

    /**
     * \brief           Computes the essential matrix from the fundamental matrix
     *
     * \param _K1       Input left intrinsic matrix
     * \param _K2       Input right intrinsic matrix
     * \param _F        Input fundamental matrix
     * \return          Output essential matrix
     **/
    Eigen::Matrix3f F2E(const Eigen::Matrix3f& _K1, const Eigen::Matrix3f& _K2, const Eigen::Matrix3f& _F);

    /**
     * \brief               Computes the essential matrix from the fundamental matrix
     *
     * \param _pSensorModel Input sensor model
     * \param _F            Input fundamental matrix
     * \return              Output essential matrix
     **/
    Eigen::Matrix3f F2E(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Matrix3f& _F);

    /**
     * \brief           Computes the epipolar line
     *
     * \param _F        Input fundamental matrix
     * \param _pt       Input 2D image point in the first (left) image
     * \return          Output coefficients of the line equation ax + by + c = 0, where a = [0], b = [1], c = [2]
     **/
    Eigen::Vector3f epi_line(const Eigen::Matrix3f& _F, const CycPoint& _pt);

    /**
     * \brief                   Computes epipolar lines
     *
     * \param _F                Input fundamental matrix
     * \param _pts               Input 2D image points in the first (left) image
     * \param _out_epi_lines    Output coefficients of the line equation ax + by + c = 0 for each given point in _pts, where a = [0], b = [1], c = [2]
     **/
    void epi_lines(const Eigen::Matrix3f& _F, const CycPoints& _pts, std::vector<Eigen::Vector3f>& _out_epi_lines);

    void epi_lines(const CPinholeCameraSensorModel* _pSensorModel, 
        const Eigen::Matrix4f& _cam_pose_1,
        const Eigen::Matrix4f& _cam_pose_2,
        const CycPoints& _pts1,
        const CycPoints& _pts2,
        std::vector<Eigen::Vector3f>& _out_epi_lines_1,
        std::vector<Eigen::Vector3f>& _out_epi_lines_2);

    /**
     * \brief               Computes the epipole (the projection of the origin of _cam_second on the image plane of _cam_first)
     *
     * \param _pSensorModel Input sensor model
     * \param _pt           Input 2D image point in the first (left) image
     * \return              Output epipole in the image plane of the first camera
     **/
    CycPoint epipole(const CPinholeCameraSensorModel* _pSensorModel, const CPose& _cam_first, const CPose& _cam_second);

    /**
     * \brief               Transforms a set of voxels given a transformation matrix
     *
     * \param _voxels       Input voxels
     * \param _T            Input transformation matrix
     * \param _th           Input distance threshold [m] (filters out voxels having one of the axes bigger than th)
     * \param _out_voxels   Output transformed voxels
     **/
    void transformVoxels(const CycVoxels& _voxels,
        const Eigen::Matrix4f& _T,
        const float& _th,
        CycVoxels& _out_voxels);

    /**
     * \brief           Calculates the reprojection error between corresponding points in two camera frames and their respective voxel (3D point)
     *
     * \param _pt1      Input point for frame 1
     * \param _pt2      Input point for frame 2
     * \param _P1       Input projection matrix for camera 1
     * \param _P2       Input projection matrix for camera 2
     * \param _voxel    Input voxel calculated from _pt1 and _pt2
     * \return          Reprojection error
     **/
    float getReprojectionErr(const CycPoint& _pt1,
        const CycPoint& _pt2,
        const Eigen::MatrixXf& _P1,
        const Eigen::MatrixXf& _P2,
        const CycVoxel& _voxel);

    /**
     * \brief           Calculates the reprojection error between sets of corresponding points in two camera frames and their respective voxels (3D points)
     *
     * \param _pts1     Input points for frame 1
     * \param _pts2     Input points for frame 2
     * \param _P1       Input projection matrix for camera 1
     * \param _P2       Input projection matrix for camera 2
     * \return          Reprojection error
     **/
    float getReprojectionErr(const CycPoints& _pts1,
        const CycPoints& _pts2,
        const Eigen::MatrixXf& _P1,
        const Eigen::MatrixXf& _P2);

    /**
     * \brief               Calculates the orthogonal projection error between corresponding points in two camera frames and their respective epipolar lines
     *
     * \param _pt1          Input point for frame 1
     * \param _pt2          Input point for frame 2
     * \param _epi_line_1   Epipolar lines for camera 1 calculated from points in camera 2
     * \param _epi_line_2   Epipolar lines for camera 2 calculated from points in camera 1
     * \return              Orthogonal projection error
     **/
    float getOrthogonalErr(const CycPoint& _pt1,
        const CycPoint& _pt2,
        const Eigen::Vector3f& _epi_line_1,
        const Eigen::Vector3f& _epi_line_2);

    /**
     * \brief               Calculates the orthogonal projection error between sets of corresponding points in two camera frames and their respective sets of epipolar lines
     *
     * \param _pts1         Input point for frame 1
     * \param _pts2         Input point for frame 2
     * \param _epi_lines_1  Epipolar lines for camera 1 calculated from points in camera 2
     * \param _epi_lines_2  Epipolar lines for camera 2 calculated from points in camera 1
     * \return              Orthogonal projection error
     **/
    float getOrthogonalErr(const CycPoints& _pts1,
        const CycPoints& _pts2,
        const std::vector<Eigen::Vector3f>& _epi_lines_1,
        const std::vector<Eigen::Vector3f>& _epi_lines_2);
};

#endif /* CProjectiveGeometry_H_ */
