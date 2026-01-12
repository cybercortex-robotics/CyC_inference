// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CGeometry.h"

#include <Eigen/LU>
#include <Eigen/Cholesky>

namespace CGeometry
{
    float euclidean_dist(const Eigen::Vector2f& _v1, const Eigen::Vector2f& _v2)
    {
        return sqrtf((_v2.x() - _v1.x()) * (_v2.x() - _v1.x()) + (_v2.y() - _v1.y()) * (_v2.y() - _v1.y()));
    }

    float euclidean_dist(const Eigen::Vector3f& _v1, const Eigen::Vector3f& _v2)
    {
        return sqrtf((_v2.x() - _v1.x()) * (_v2.x() - _v1.x()) + (_v2.y() - _v1.y()) * (_v2.y() - _v1.y()) + (_v2.z() - _v1.z()) * (_v2.z() - _v1.z()));
    }

    float euclidean_dist(const Eigen::Vector4f& _v1, const Eigen::Vector4f& _v2)
    {
        return sqrtf((_v2.x() - _v1.x()) * (_v2.x() - _v1.x()) + (_v2.y() - _v1.y()) * (_v2.y() - _v1.y()) + (_v2.z() - _v1.z()) * (_v2.z() - _v1.z()) + (_v2.w() - _v1.w()) * (_v2.w() - _v1.w()));
    }

    float abs_dist(const Eigen::Vector2f& _pt1, const Eigen::Vector2f& _pt2)
    {
        Eigen::Vector2f v = _pt1 - _pt2;
        return v.dot(v);
    }

    float abs_dist(const Eigen::Vector3f& _pt1, const Eigen::Vector3f& _pt2)
    {
        Eigen::Vector3f v = _pt1 - _pt2;
        return v.dot(v);
    }

    float orthogonal_dist(const Eigen::Vector3f& _line_coeffs, const Eigen::Vector2f& _pt)
    {
        return fabsf(_line_coeffs.x() * _pt.x() + _line_coeffs.y() * _pt.y() + _line_coeffs.z()) / sqrtf(_line_coeffs.x() * _line_coeffs.x() + _line_coeffs.y() * _line_coeffs.y());
    }

    float mag(const Eigen::Vector2f& _vect)
    {
        return sqrtf(_vect.x() * _vect.x() + _vect.y() * _vect.y());
    }

    float mag(const Eigen::Vector3f& _vect)
    {
        return sqrtf(_vect.x() * _vect.x() + _vect.y() * _vect.y() + _vect.z() * _vect.z());
    }

    float mag(const Eigen::Vector4f& _vect)
    {
        return sqrtf(_vect.x() * _vect.x() + _vect.y() * _vect.y() + _vect.z() * _vect.z() + _vect.w() * _vect.w());
    }

    float mag(const CycVoxels& _vxs)
    {
        float mag = 0.f;
        for (const auto& vx : _vxs)
            mag += CGeometry::mag(vx.pt3d);
        return mag / _vxs.size();
    }

    float point_on_line(const Eigen::Vector3f& _line_coeffs, const Eigen::Vector2f& _pt)
    {
        Eigen::Vector3f pt_hom(_pt.x(), _pt.y(), 1.f);
        return pt_hom.transpose() * _line_coeffs;
    }

    Eigen::Vector2f intersection(const Eigen::Vector3f& _line_coeffs_1, const Eigen::Vector3f& _line_coeffs_2)
    {
        Eigen::Vector3f intersection = _line_coeffs_1.cross(_line_coeffs_2);
        return Eigen::Vector2f{ intersection.x() / intersection.z(), intersection.y() / intersection.z() };
    }

    Eigen::Vector3f points2line(const Eigen::Vector2f& _pt1, const Eigen::Vector2f& _pt2)
    {
        Eigen::Vector3f pt1_hom(_pt1.x(), _pt1.y(), 1.f);
        Eigen::Vector3f pt2_hom(_pt2.x(), _pt2.y(), 1.f);
        return pt1_hom.cross(pt2_hom);
    }

    Eigen::Vector2f orthogonal_projection(const Eigen::Vector3f& _line_coeffs, const Eigen::Vector2f& _pt)
    {
        float m = -_line_coeffs.x() / _line_coeffs.y(); // slope
        float b = -_line_coeffs.z() / _line_coeffs.y(); // intercept
        float mm = m * m;

        return Eigen::Vector2f {
            (_pt.x() + m * _pt.y() - m * b) / (1.f + mm),
            (m * _pt.x() + mm * _pt.y() + b) / (1.f + mm)
        };
    }
}


