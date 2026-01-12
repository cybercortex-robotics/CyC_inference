// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CQuaternion.h"

CQuaternion::CQuaternion()
{
    this->m_x = 0.f;
    this->m_y = 0.f;
    this->m_z = 0.f;
    this->m_w = 1.f;
}

CQuaternion::CQuaternion(const Eigen::Vector4f _q)
{
    this->m_x = _q.x();
    this->m_y = _q.y();
    this->m_z = _q.z();
    this->m_w = _q.w();
}

CQuaternion::CQuaternion(const float _x, const float _y, const float _z, const float _w)
{
    this->m_x = _x;
    this->m_y = _y;
    this->m_z = _z;
    this->m_w = _w;
}

CQuaternion::CQuaternion(const Eigen::Matrix3f _R)
{
    *this = CQuaternion(rotmat2quaternion(_R));
}

CQuaternion::CQuaternion(const float _roll, const float _pitch, const float _yaw)
{
    *this = CQuaternion(euler2quaternion(_roll, _pitch, _yaw));
}

void CQuaternion::update(const Eigen::Vector4f _q)
{
    this->m_x = _q.x();
    this->m_y = _q.y();
    this->m_z = _q.z();
    this->m_w = _q.w();
}

void CQuaternion::update(const float _x, const float _y, const float _z, const float _w)
{
    this->m_x = _x;
    this->m_y = _y;
    this->m_z = _z;
    this->m_w = _w;
}

void CQuaternion::update(const Eigen::Matrix3f _R)
{
    *this = rotmat2quaternion(_R);
}

void CQuaternion::update(const float _roll, const float _pitch, const float _yaw)
{
    *this = euler2quaternion(_roll, _pitch, _yaw);
}

const CQuaternion CQuaternion::operator*(const CQuaternion& _q) const
{
    CQuaternion qu;
    qu.m_x = this->m_w * _q.m_x + this->m_x * _q.m_w + this->m_y * _q.m_z - this->m_z * _q.m_y;
    qu.m_y = this->m_w * _q.m_y + this->m_y * _q.m_w + this->m_z * _q.m_x - this->m_x * _q.m_z;
    qu.m_z = this->m_w * _q.m_z + this->m_z * _q.m_w + this->m_x * _q.m_y - this->m_y * _q.m_x;
    qu.m_w = this->m_w * _q.m_w - this->m_x * _q.m_x - this->m_y * _q.m_y - this->m_z * _q.m_z;
    return qu;
}

const CQuaternion CQuaternion::operator/ (float _s) const
{
    CQuaternion qu = (*this);
    return CQuaternion(qu.m_x / _s, qu.m_y / _s, qu.m_z / _s, qu.m_w / _s);
}

CQuaternion CQuaternion::operator+(const CQuaternion& _q) const
{
    return *this * _q;
}

CQuaternion CQuaternion::operator-(const CQuaternion& _q) const
{
    return *this * _q.inverse();
}

float CQuaternion::dot(const CQuaternion& q1, const CQuaternion& q2)
{
    return q1.m_x * q2.m_x + q1.m_y * q2.m_y + q1.m_z * q2.m_z + q1.m_w * q2.m_w;
}

CQuaternion CQuaternion::inverse() const
{
    CQuaternion q = (*this);
    return q.conjugate() / CQuaternion::dot((*this), (*this));
}

CQuaternion CQuaternion::conjugate() const
{
    CQuaternion q;
    q.m_x = -this->m_x;
    q.m_y = -this->m_y;
    q.m_z = -this->m_z;
    q.m_w = this->m_w;
    return q;
}

Eigen::Matrix3f CQuaternion::to_rotmat() const
{
    return quaternion2rotmat(this->m_x, this->m_y, this->m_z, this->m_w);
}

Eigen::Vector3f CQuaternion::to_euler_ZYX() const
{
    return quaternion2euler_ZYX(this->m_x, this->m_y, this->m_z, this->m_w);
}

Eigen::Vector4f CQuaternion::to_vector() const
{
    return Eigen::Vector4f(m_x, m_y, m_z, m_w);
}

const float CQuaternion::x() const
{
    return m_x;
}

const float CQuaternion::y() const
{
    return m_y;
}

const float CQuaternion::z() const
{
    return m_z;
}

const float CQuaternion::w() const
{
    return m_w;
}

CQuaternion CQuaternion::rotmat2quaternion(const Eigen::Matrix3f _R)
{
    Eigen::Vector4f q;

    q.w() = sqrtf(fmaxf(0.f, 1.f + _R(0, 0) + _R(1, 1) + _R(2, 2))) * 0.5f;
    q.x() = sqrtf(fmaxf(0.f, 1.f + _R(0, 0) - _R(1, 1) - _R(2, 2))) * 0.5f;
    q.y() = sqrtf(fmaxf(0.f, 1.f - _R(0, 0) + _R(1, 1) - _R(2, 2))) * 0.5f;
    q.z() = sqrtf(fmaxf(0.f, 1.f - _R(0, 0) - _R(1, 1) + _R(2, 2))) * 0.5f;

    if ((_R(2, 1) - _R(1, 2)) >= 0)
        q.x() = fabsf(q.x());
    else
        q.x() = -fabsf(q.x());

    if ((_R(0, 2) - _R(2, 0)) >= 0)
        q.y() = fabsf(q.y());
    else
        q.y() = -fabsf(q.y());

    if ((_R(1, 0) - _R(0, 1)) >= 0)
        q.z() = fabsf(q.z());
    else
        q.z() = -fabsf(q.z());

    return CQuaternion(q);
}

Eigen::Matrix3f CQuaternion::quaternion2rotmat(const float _x, const float _y, const float _z, const float _w)
{
    Eigen::Matrix3f R;

    float xx = _x * _x;
    float xy = _x * _y;
    float xz = _x * _z;
    float xw = _x * _w;

    float yy = _y * _y;
    float yz = _y * _z;
    float yw = _y * _w;

    float zz = _z * _z;
    float zw = _z * _w;

    R(0, 0) = 1.f - 2.f * (yy + zz);
    R(0, 1) = 2.f * (xy - zw);
    R(0, 2) = 2.f * (xz + yw);

    R(1, 0) = 2.f * (xy + zw);
    R(1, 1) = 1.f - 2.f * (xx + zz);
    R(1, 2) = 2.f * (yz - xw);

    R(2, 0) = 2.f * (xz - yw);
    R(2, 1) = 2.f * (yz + xw);
    R(2, 2) = 1.f - 2.f * (xx + yy);

    return R;
}

Eigen::Matrix3f CQuaternion::quaternion2rotmat(const Eigen::Vector4f _q)
{
    return quaternion2rotmat(_q.x(), _q.y(), _q.z(), _q.w());
}

CQuaternion CQuaternion::euler2quaternion(const float _roll, const float _pitch, const float _yaw)
{
    float cy = cosf(_yaw * 0.5f);
    float sy = sinf(_yaw * 0.5f);
    float cp = cosf(_pitch * 0.5f);
    float sp = sinf(_pitch * 0.5f);
    float cr = cosf(_roll * 0.5f);
    float sr = sinf(_roll * 0.5f);

    Eigen::Vector4f q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return CQuaternion(q);
}

Eigen::Vector3f CQuaternion::quaternion2euler_ZYX(const float _x, const float _y, const float _z, const float _w)
{
    Eigen::Vector3f angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2.f * (_w * _x + _y * _z);
    float cosr_cosp = 1.f - 2.f * (_x * _x + _y * _y);
    angles.x() = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.f * (_w * _y - _z * _x);
    if (std::abs(sinp) >= 1.f)
        angles.y() = std::copysign((float)M_PI / 2.f, sinp); // use 90 degrees if out of range
    else
        angles.y() = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.f * (_w * _z + _x * _y);
    float cosy_cosp = 1.f - 2.f * (_y * _y + _z * _z);
    angles.z() = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Eigen::Vector3f CQuaternion::quaternion2euler_ZYX(const Eigen::Vector4f _q)
{
    return quaternion2euler_ZYX(_q.x(), _q.y(), _q.z(), _q.w());
}
