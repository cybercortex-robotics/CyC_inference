// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CProjectiveGeometry.h"
#include "CTriangulation.h"
#include <opencv2/core/eigen.hpp>

namespace CProjectiveGeometry
{
    Eigen::Matrix3f vector2skewsymmetric(const Eigen::Vector3f& _t)
    {
        Eigen::Matrix3f Sb;
        Sb << 0.f, -_t(2), _t(1), _t(2), 0.f, -_t(0), -_t(1), _t(0), 0.f;
        return Sb;
    }

    Eigen::Matrix4f addScaleFactor(const Eigen::Matrix4f& _T, const float& _scale)
    {
        Eigen::Matrix4f T_scale = Eigen::Matrix4f::Identity();
        T_scale.block(0, 0, 3, 3) = _T.block(0, 0, 3, 3);
        T_scale(0, 3) = _T(0, 3) * _scale;
        T_scale(1, 3) = _T(1, 3) * _scale;
        T_scale(2, 3) = _T(2, 3) * _scale;
        return T_scale;
    }

    float getScaleFactor(const Eigen::Vector4f& _trans_first_1, const Eigen::Vector4f& _trans_first_2,
        const Eigen::Vector4f& _trans_second_1, const Eigen::Vector4f& _trans_second_2)
    {
        float ed_given = CGeometry::euclidean_dist(_trans_first_1, _trans_first_2);
        float ed_triang = CGeometry::euclidean_dist(_trans_second_1, _trans_second_2);

        return ed_given / ed_triang;
    }

    float getScaleFactor(const CycVoxels& _vxs_real, const CycVoxels& _vxs_estimated)
    {
        assert(_vxs_real.size() != _vxs_estimated.size());

        float scale = 0.f;
        CyC_INT count = 0;

        for (size_t j = 0; j < _vxs_estimated.size() - 1; j++)
        {
            for (size_t k = j + 1; k < _vxs_estimated.size(); k++)
            {
                scale += getScaleFactor(_vxs_real[j].pt3d, _vxs_real[k].pt3d, _vxs_estimated[j].pt3d, _vxs_estimated[k].pt3d);
                count++;
            }
        }

        return scale / count;
    }

    // Calculates the camera projection matrix based on the camera intrinsic matrix and the camera's extrinsics
    Pmatrix KT2P(const Eigen::Matrix3f& _K, const Eigen::Matrix4f& _T)
    {
        Pmatrix P(3, 4);
        return _K * _T.block(0, 0, 3, 4);
    }

    Eigen::Vector2f distort(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Vector2f& _pt_norm_undist)
    {
        // Camera intrinsics
        float fx = _pSensorModel->fx_px();
        float fy = _pSensorModel->fy_px();
        float cx = _pSensorModel->cx();
        float cy = _pSensorModel->cy();
        float k1 = _pSensorModel->D_k1();
        float k2 = _pSensorModel->D_k2();
        float p1 = _pSensorModel->D_p1();
        float p2 = _pSensorModel->D_p2();

        float x = _pt_norm_undist.x();
        float y = _pt_norm_undist.y();

        // Apply RadTan Distortion
        float r2 = x * x + y * y;
        float r4 = r2 * r2;

        // Radial distortion
        float radial = (1.0f + k1 * r2 + k2 * r4);

        // Tangential distortion
        float dx = 2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x);
        float dy = p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y;

        float x_distorted = x * radial + dx;
        float y_distorted = y * radial + dy;

        return Eigen::Vector2f{ x_distorted, y_distorted };
    }

    Eigen::Vector2f undistort_px(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Vector2f& _pt_dist)
    {
        Eigen::Vector2f pt_norm_dist = {
            (_pt_dist.x() - _pSensorModel->cx()) / _pSensorModel->fx_px(),
            (_pt_dist.y() - _pSensorModel->cy()) / _pSensorModel->fy_px()
        };
        Eigen::Vector2f pt_norm_undist = undistort(_pSensorModel, pt_norm_dist);

        Eigen::Vector2f pt_undist{ pt_norm_undist.x() * _pSensorModel->fx_px()  + _pSensorModel->cx(),
            pt_norm_undist.y() * _pSensorModel->fy_px() + _pSensorModel->cy()
        };
        return pt_undist;
    }

    Eigen::Vector2f undistort(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Vector2f& _pt_norm_dist)
    {
        Eigen::Vector2f pt_norm_undist = _pt_norm_dist; // Initial guess
        for (int i = 0; i < 5; i++)
        {
            Eigen::Vector2f pt_estimate = CProjectiveGeometry::distort(_pSensorModel, pt_norm_undist);
            pt_norm_undist += (_pt_norm_dist - pt_estimate);
        }
        return pt_norm_undist;
    }

    CycPoint project(const CPinholeCameraSensorModel* _pSensorModel, const CycVoxel& _voxel_C)
    {
        // Safety check: point must be in front of the camera
        if (_voxel_C.pt3d.z() <= 0)
            return CycPoint{ Eigen::Vector2f{-1, -1}, _voxel_C.pt3d.z(), _voxel_C.id };

        // Camera intrinsics
        float fx = _pSensorModel->fx_px();
        float fy = _pSensorModel->fy_px();
        float cx = _pSensorModel->cx();
        float cy = _pSensorModel->cy();

        // Normalize coordinates (Ideal Pinhole)
        Eigen::Vector2f pt_norm_undist{ _voxel_C.pt3d.x() / _voxel_C.pt3d.z(), _voxel_C.pt3d.y() / _voxel_C.pt3d.z() };

        // Distort point
        Eigen::Vector2f pt_norm_dist = distort(_pSensorModel, pt_norm_undist);

        // Map to Pixel Coordinates (using K)
        float u = fx * pt_norm_dist.x() + cx;
        float v = fy * pt_norm_dist.y() + cy;

        return CycPoint{ Eigen::Vector2f{u, v}, _voxel_C.pt3d.z(), _voxel_C.id };;
    }
    
    CycPoint project(const CPinholeCameraSensorModel* _pSensorModel, const CPose& _cam_pose_W, const CycVoxel& _voxel_W)
    {
        return project(_pSensorModel, _cam_pose_W.transform(), _voxel_W);
    }

    CycPoint project(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Matrix4f& _cam_pose_W, const CycVoxel& _voxel_W)
    {
        Eigen::Vector4f vx_C = _cam_pose_W.inverse() * _voxel_W.pt3d;
        return project(_pSensorModel, vx_C);
    }

    void project(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Matrix4f& _cam_pose_W, const CycVoxels& _voxels_W, CycPoints& _out_pts)
    {
        _out_pts.clear();
        for (size_t i = 0; i < _voxels_W.size(); ++i)
            _out_pts.emplace_back(project(_pSensorModel, _cam_pose_W, _voxels_W[i]));
    }

    void project(const CPinholeCameraSensorModel* _pSensorModel, const CycVoxels& _voxels_C, CycPoints& _out_pts)
    {
        _out_pts.clear();
        for (size_t i = 0; i < _voxels_C.size(); ++i)
            _out_pts.emplace_back(project(_pSensorModel, _voxels_C[i]));
    }

    void project(const CPinholeCameraSensorModel* _pSensorModel, const CPose& _cam_pose_W, const CycVoxels& _voxels_W, CycPoints& _out_pts)
    {
        project(_pSensorModel, _cam_pose_W.transform(), _voxels_W, _out_pts);
    }

    Eigen::Matrix4f invertT(const Eigen::Matrix4f& _T)
    {
        Eigen::Matrix4f T_inv;
        //Eigen::Matrix3f R;
        //Eigen::Vector3f t;

        Eigen::Matrix3f R = _T.block<3, 3>(0, 0).cast<float>();
        Eigen::Vector3f t = _T.block<3, 1>(0, 3).cast<float>();

        // Update rotation
        T_inv.block<3, 3>(0, 0) = R.transpose();

        // Update position. Solve RC+r=0, where C is the optical center
        T_inv.block<3, 1>(0, 3) = -R.transpose() * t;

        T_inv.row(3) << 0.f, 0.f, 0.f, 1.f;

        return T_inv;
    }

    Eigen::Matrix3f Rt2E(const Eigen::Matrix3f& _R, const Eigen::Vector3f& _t)
    {
        // https://stackoverflow.com/questions/27031425/calculating-essential-matrix-using-rotation-translation-and-camera-parameters
        // https://www.youtube.com/watch?v=zX5NeY-GTO0&t=1935s
        Eigen::Matrix3f Sb = vector2skewsymmetric(_t);
        return Sb * _R;
    }

    Eigen::Matrix3f T2E(const Eigen::Matrix4f& _T)
    {
        return Rt2E(_T.block(0, 0, 3, 3), _T.block(0, 3, 3, 1));
    }

    void E2T(const Eigen::Matrix3f& _E, Eigen::Matrix4f& _T1, Eigen::Matrix4f& _T2, Eigen::Matrix4f& _T3, Eigen::Matrix4f& _T4)
    {
        // Assumes input E is a rank 2 matrix, with equal singular values
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(_E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix3f& U = svd.matrixU(),
            & V = svd.matrixV();
        Eigen::Matrix3f W;

        // Find rotation, translation
        W.setZero();
        W(0, 1) = -1.0f;
        W(1, 0) = 1.0f;
        W(2, 2) = 1.0f;

        // Rotation
        Eigen::Matrix3f R1 = U * W * V.transpose();
        Eigen::Matrix3f R2 = U * W.transpose() * V.transpose();

        if (R1.determinant() < 0.f)
            R1 *= -1.f;
        if (R2.determinant() < 0.f)
            R2 *= -1.f;

        _T1.block(0, 0, 3, 3) = R1;
        _T2.block(0, 0, 3, 3) = R1;
        _T3.block(0, 0, 3, 3) = R2;
        _T4.block(0, 0, 3, 3) = R2;

        // Translation
        _T1.block(0, 3, 3, 3) = U.col(2);
        _T2.block(0, 3, 3, 3) = -U.col(2);
        _T3.block(0, 3, 3, 3) = U.col(2);
        _T4.block(0, 3, 3, 3) = -U.col(2);

        // Last row
        _T1.row(3) << 0.f, 0.f, 0.f, 1.f;
        _T2.row(3) << 0.f, 0.f, 0.f, 1.f;
        _T3.row(3) << 0.f, 0.f, 0.f, 1.f;
        _T4.row(3) << 0.f, 0.f, 0.f, 1.f;
    }

    void E2T(const Eigen::Matrix3f& _E, std::vector<Eigen::Matrix4f>& _Ts)
    {
        // https://gist.github.com/jensenb/8668000
        _Ts.clear();

        std::vector<Eigen::Matrix3f> Rs;
        std::vector<Eigen::Vector3f> ts;
        E2Rt(_E, Rs, ts);

        if (Rs.size() < 2 || ts.size() < 2)
        {
            spdlog::error("E2T: not enough projection solutions");
            return;
        }

        Eigen::Matrix4f T1, T2, T3, T4;

        // Rotation
        T1.block(0, 0, 3, 3) = Rs[0];
        T2.block(0, 0, 3, 3) = Rs[0];
        T3.block(0, 0, 3, 3) = Rs[1];
        T4.block(0, 0, 3, 3) = Rs[1];

        // Translation
        T1.col(3) << ts[0](0), ts[0](1), ts[0](2), 1.f;
        T2.col(3) << ts[1](0), ts[1](1), ts[1](2), 1.f;
        T3.col(3) << ts[0](0), ts[0](1), ts[0](2), 1.f;
        T4.col(3) << ts[1](0), ts[1](1), ts[1](2), 1.f;

        // Last row
        T1.row(3) << 0.f, 0.f, 0.f, 1.f;
        T2.row(3) << 0.f, 0.f, 0.f, 1.f;
        T3.row(3) << 0.f, 0.f, 0.f, 1.f;
        T4.row(3) << 0.f, 0.f, 0.f, 1.f;

        _Ts.emplace_back(T1);
        _Ts.emplace_back(T2);
        _Ts.emplace_back(T3);
        _Ts.emplace_back(T4);
    }

    void E2Rt(const Eigen::Matrix3f& _E, std::vector<Eigen::Matrix3f>& _Rs, std::vector<Eigen::Vector3f>& _ts)
    {
        _Rs.clear();
        _ts.clear();

        // Assumes input E is a rank 2 matrix, with equal singular values
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(_E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix3f& U = svd.matrixU(),
            & V = svd.matrixV();
        Eigen::Matrix3f W, Z;

        W << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f;
        Z << 0.f, 1.f, 0.f, -1.f, 0.f, 0.f, 0.f, 0.f, 0.f;

        // Rotation
        Eigen::Matrix3f R1 = U * W * V.transpose();
        Eigen::Matrix3f R2 = U * W.transpose() * V.transpose();
        Eigen::Matrix3f T = U * Z * U.transpose();

        if (R1.determinant() < 0.f)
            R1 *= -1.f;
        if (R2.determinant() < 0.f)
            R2 *= -1.f;

        _Rs.emplace_back(R1);
        _Rs.emplace_back(R2);
        _ts.emplace_back(Eigen::Vector3f{ T(2, 1), -T(2, 0), T(1, 0) });
        _ts.emplace_back(Eigen::Vector3f{ -T(2, 1), T(2, 0), -T(1, 0) });
    }

    Eigen::Matrix3f E2F(const Eigen::Matrix3f& _K1, const Eigen::Matrix3f& _K2, const Eigen::Matrix3f& _E)
    {
        return (_K2.transpose()).inverse() * _E * _K1.inverse();
    }

    Eigen::Matrix3f E2F(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Matrix3f& _E)
    {
        return E2F(_pSensorModel->K(), _pSensorModel->K(), _E);
    }

    Eigen::Matrix3f F2E(const Eigen::Matrix3f& _K1, const Eigen::Matrix3f& _K2, const Eigen::Matrix3f& _F)
    {
        return _K2.transpose() * _F * _K1;
    }

    Eigen::Matrix3f F2E(const CPinholeCameraSensorModel* _pSensorModel, const Eigen::Matrix3f& _F)
    {
        return F2E(_pSensorModel->K(), _pSensorModel->K(), _F);
    }

    Eigen::Vector3f epi_line(const Eigen::Matrix3f& _F, const CycPoint& _pt)
    {
        Eigen::Vector3f pt_hom(_pt.pt2d.x(), _pt.pt2d.y(), 1.f);
        return _F * pt_hom;
    }

    void epi_lines(const Eigen::Matrix3f& _F, const CycPoints& _pts, std::vector<Eigen::Vector3f>& _out_epi_lines)
    {
        _out_epi_lines.clear();

        for (size_t k = 0; k < _pts.size(); ++k)
            _out_epi_lines.emplace_back(CProjectiveGeometry::epi_line(_F, _pts[k]));
    }

    void epi_lines(const CPinholeCameraSensorModel* _pSensorModel, 
        const Eigen::Matrix4f& _cam_pose_1,
        const Eigen::Matrix4f& _cam_pose_2,
        const CycPoints& _pts1,
        const CycPoints& _pts2,
        std::vector<Eigen::Vector3f>& _out_epi_lines_1,
        std::vector<Eigen::Vector3f>& _out_epi_lines_2)
    {
        assert(_pts1.size() != _pts2.size());

        _out_epi_lines_1.clear();
        _out_epi_lines_2.clear();

        // Compute essential matrices
        Eigen::Matrix4f T_given_1 = CProjectiveGeometry::invertT(_cam_pose_1) * _cam_pose_2;
        Eigen::Matrix4f T_given_2 = CProjectiveGeometry::invertT(_cam_pose_2) * _cam_pose_1;
        Eigen::Matrix3f E_cam1_2_cam2 = CProjectiveGeometry::T2E(T_given_1);
        Eigen::Matrix3f E_cam2_2_cam1 = CProjectiveGeometry::T2E(T_given_2);

        // Calculate fundamental matrices (mapping between images)
        Eigen::Matrix3f F_cam2_2_cam1 = CProjectiveGeometry::E2F(_pSensorModel, E_cam2_2_cam1);
        Eigen::Matrix3f F_cam1_2_cam2 = CProjectiveGeometry::E2F(_pSensorModel, E_cam1_2_cam2);

        // Calculate the epipolar lines
        CProjectiveGeometry::epi_lines(F_cam1_2_cam2, _pts2, _out_epi_lines_1);
        CProjectiveGeometry::epi_lines(F_cam2_2_cam1, _pts1, _out_epi_lines_2);
    }

    CycPoint epipole(const CPinholeCameraSensorModel* _pSensorModel, const CPose& _cam_first, const CPose& _cam_second)
    {
        Eigen::Vector3f o2 = _cam_second.translation_3x1();
        CycVoxel o2_hom(Eigen::Vector4f{ o2.x(), o2.y(), o2.z(), 1.f });
        return CProjectiveGeometry::project(_pSensorModel, _cam_first.transform(), o2_hom);
    }

    void transformVoxels(const CycVoxels& _voxels,
        const Eigen::Matrix4f& _T,
        const float& _th_proximity,
        const float& _th_far,
        CycVoxels& _out_voxels)
    {
        CycVoxels result;
        for (std::size_t i = 0; i < _voxels.size(); ++i)
        {
            const Eigen::Vector4f* p = &_voxels[i].pt3d;
            if (p->x() < _th_far && p->y() < _th_far && p->z() < _th_far && fabs(p->x()) > _th_proximity && fabs(p->y()) > _th_proximity && fabs(p->z()) > _th_proximity)
                result.emplace_back(CycVoxel{ _T * _voxels[i].pt3d, _voxels[i].id });
        }
        _out_voxels = result;
    }
    
    void clipVoxels(const CycVoxels& _voxels, 
        CycVoxels& _out_voxels, 
        const Eigen::Vector3f& _neg_range, 
        const Eigen::Vector3f& _pos_range)
    {
        CycVoxels result;
        for (std::size_t i = 0; i < _voxels.size(); ++i)
        {
            const Eigen::Vector4f* p = &_voxels[i].pt3d;
            if (p->x() >= _neg_range.x() && p->y() >= _neg_range.y() && p->z() >= _neg_range.z() &&
                p->x() <= _pos_range.x() && p->y() <= _pos_range.y() && p->z() <= _pos_range.z())
                result.emplace_back(_voxels[i]);
        }
        _out_voxels = result;
    }

    float getReprojectionErr(const CPinholeCameraSensorModel* _pSensorModel, 
        const CycPoint& _pt1,
        const CycPoint& _pt2,
        const Eigen::MatrixXf& _P1,
        const Eigen::MatrixXf& _P2,
        const CycVoxel& _pt3d)
    {
        CycPoint pt2d_reproj_1 = CProjectiveGeometry::project(_pSensorModel, _pt3d);
        CycPoint pt2d_reproj_2 = CProjectiveGeometry::project(_pSensorModel, _pt3d);

        float Ed = CGeometry::euclidean_dist(_pt1.pt2d, pt2d_reproj_1.pt2d);
        Ed += CGeometry::euclidean_dist(_pt2.pt2d, pt2d_reproj_2.pt2d);

        return Ed / 2.f;
    }
    
    float getReprojectionErr(const CPinholeCameraSensorModel* _pSensorModel, 
        const CycPoints& _pts1,
        const CycPoints& _pts2,
        const Eigen::MatrixXf& _P1,
        const Eigen::MatrixXf& _P2)
    {
        assert(_pts1.size() != _pts2.size());

        float Err_sum = 0.f;

        CycVoxels voxels;
        CTriangulation::TriangulatePoints(_pts1, _pts2, _P1, _P2, voxels);

        for (size_t k = 0; k < _pts1.size(); ++k)
            Err_sum += getReprojectionErr(_pSensorModel, _pts1[k], _pts2[k], _P1, _P2, voxels[k]);

        return Err_sum / _pts1.size();
    }

    float getOrthogonalErr(const CycPoint& _pt1,
        const CycPoint& _pt2,
        const Eigen::Vector3f& _epi_line_1,
        const Eigen::Vector3f& _epi_line_2)
    {
        float pt_orth_epi_1 = CGeometry::orthogonal_dist(_epi_line_1, _pt1.pt2d);
        float pt_orth_epi_2 = CGeometry::orthogonal_dist(_epi_line_2, _pt2.pt2d);

        return (pt_orth_epi_1 + pt_orth_epi_2) / 2.f;
    }

    float getOrthogonalErr(const CycPoints& _pts1,
        const CycPoints& _pts2,
        const std::vector<Eigen::Vector3f>& _epi_lines_1,
        const std::vector<Eigen::Vector3f>& _epi_lines_2)
    {
        assert(_pts1.size() != _pts2.size() != _epi_lines_1.size() != _epi_lines_2.size());

        float err = 0.f;

        for (size_t i = 0; i < _pts1.size(); ++i)
            err += getOrthogonalErr(_pts1[i], _pts2[i], _epi_lines_1[i], _epi_lines_2[i]);

        return err / _pts1.size();
    }
}
