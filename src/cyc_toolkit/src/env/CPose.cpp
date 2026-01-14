// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CPose.h"

CPose::CPose(int _id) :
    m_ID(_id)
{
    this->m_Transform = Eigen::Matrix4f::Identity();
}

CPose::CPose(const Eigen::Matrix4f& _pose, int _id) :
    m_ID(_id)
{
    this->update(_pose);
}

CPose::CPose(const float& _trans_x, const float& _trans_y, const float& _trans_z,
    const float& _rot_x, const float& _rot_y, const float& _rot_z, int _id) :
    m_ID(_id)
{
    this->update(_trans_x, _trans_y, _trans_z, _rot_x, _rot_y, _rot_z);
}

CPose::CPose(const float& _trans_x, const float& _trans_y, const float& _trans_z,
    const float& _rot_quat_x, const float& _rot_quat_y, const float& _rot_quat_z, const float& _rot_quat_w, int _id) :
    m_ID(_id)
{
    this->update(_trans_x, _trans_y, _trans_z, _rot_quat_x, _rot_quat_y, _rot_quat_z, _rot_quat_w);
}

CPose::CPose(const Eigen::Vector3f& _trans, const Eigen::Vector3f& _rot_euler, int _id) :
    m_ID(_id)
{
    this->update(_trans, _rot_euler);
}

CPose::CPose(const Eigen::Vector3f& _trans, const Eigen::Vector4f& _rot_quat, int _id) :
    m_ID(_id)
{
    this->update(_trans, _rot_quat);
}

CPose::CPose(const std::string& _str, int _id) :
    m_ID(_id)
{
    this->update(_str);
}

CPose::CPose(const Eigen::Vector3f& _trans, const Eigen::Matrix3f& _rot_mat, int _id) :
    m_ID(_id)
{
    this->update(_trans, _rot_mat);
}

const int CPose::getID() const
{
    return m_ID;
}

const void CPose::setID(int _id)
{
    m_ID = _id;
}

CPose CPose::operator*(const CPose& _pose) const
{
    return CPose(this->transform() * _pose.m_Transform, _pose.m_ID);
}

CPose CPose::operator+(const CPose& _pose) const
{
    Eigen::Vector3f trans_diff = this->translation_3x1() + _pose.translation_3x1();
    CQuaternion rot_diff = CQuaternion(this->rotation_quat()) + CQuaternion(_pose.rotation_quat());
    return CPose(trans_diff, rot_diff.to_vector());
}

CPose CPose::operator-(const CPose& _pose) const
{
    Eigen::Vector3f trans_diff = this->translation_3x1() - _pose.translation_3x1();
    CQuaternion rot_diff = this->rotation_quat().conjugate() * _pose.rotation_quat();
    return CPose(trans_diff, rot_diff.to_vector());
}

void CPose::update(const Eigen::Matrix4f& _pose)
{
    m_Transform = _pose;
}

void CPose::update(const float& _trans_x, const float& _trans_y, const float& _trans_z,
    const float& _rot_euler_x, const float& _rot_euler_y, const float& _rot_euler_z)
{
    m_Transform = Rt2T(_rot_euler_x, _rot_euler_y, _rot_euler_z, _trans_x, _trans_y, _trans_z);
}

void CPose::update(const float& _trans_x, const float& _trans_y, const float& _trans_z,
    const float& _rot_quat_x, const float& _rot_quat_y, const float& _rot_quat_z, const float& _rot_quat_w)
{
    m_Transform = Rt2T(_rot_quat_x, _rot_quat_y, _rot_quat_z, _rot_quat_w, _trans_x, _trans_y, _trans_z);
}

void CPose::update(const Eigen::Vector3f& _trans, const Eigen::Vector3f& _rot_euler)
{
    m_Transform.block<3, 3>(0, 0) = euler2R_ZYX(_rot_euler.x(), _rot_euler.y(), _rot_euler.z());
    m_Transform.block<3, 1>(0, 3) = _trans;
    m_Transform.row(3) << 0.f, 0.f, 0.f, 1.f;
}

void CPose::update(const Eigen::Vector3f& _trans, const Eigen::Vector4f& _rot_quat)
{
    m_Transform.block<3, 3>(0, 0) = CQuaternion::quaternion2rotmat(_rot_quat);
    m_Transform.block<3, 1>(0, 3) = _trans;
    m_Transform.row(3) << 0.f, 0.f, 0.f, 1.f;
}

void CPose::update(const Eigen::Vector3f& _trans, const Eigen::Matrix3f& _rot_mat)
{
    m_Transform.block<3, 3>(0, 0) = _rot_mat;
    m_Transform.block<3, 1>(0, 3) = _trans;
    m_Transform.row(3) << 0.f, 0.f, 0.f, 1.f;
}

void CPose::update(const std::string& _str)
{
    std::vector<std::string> tokens;
    std::string delimiter = ",";
    CStringUtils::splitstring(_str, delimiter, tokens);

    if (tokens.size() != 6)
        return;

    this->update(std::stof(tokens[0]), std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3]), std::stof(tokens[4]), std::stof(tokens[5]));
}

void CPose::update_R(const Eigen::Matrix3f& _R)
{
    m_Transform.block<3, 3>(0, 0) = _R;
}

void CPose::update_R(const Eigen::Vector3f& _rot_euler)
{
    m_Transform.block<3, 3>(0, 0) = euler2R_ZYX(_rot_euler.x(), _rot_euler.y(), _rot_euler.z());
}

void CPose::update_R(const Eigen::Vector4f& _rot_quat)
{
    m_Transform.block<3, 3>(0, 0) = CQuaternion::quaternion2rotmat(_rot_quat);
}

void CPose::update_R(const float& _rot_euler_x, const float& _rot_euler_y, const float& _rot_euler_z)
{
    m_Transform.block<3, 3>(0, 0) = euler2R_ZYX(_rot_euler_x, _rot_euler_y, _rot_euler_z);
}

void CPose::update_R(const float& _rot_quat_x, const float& _rot_quat_y, const float& _rot_quat_z, const float& _rot_quat_w)
{
    m_Transform.block<3, 3>(0, 0) = CQuaternion::quaternion2rotmat(_rot_quat_x, _rot_quat_y, _rot_quat_z, _rot_quat_w);
}

void CPose::update_t(const Eigen::Vector3f& _trans)
{
    m_Transform.block<3, 1>(0, 3) = _trans;
}

void CPose::update_t(const float& _trans_x, const float& _trans_y, const float& _trans_z)
{
    m_Transform(0, 3) = _trans_x;
    m_Transform(1, 3) = _trans_y;
    m_Transform(2, 3) = _trans_z;
}

const Eigen::Matrix4f CPose::transform() const
{
    return m_Transform;
}

const CPose CPose::inverse() const
{
    CPose pose(this->m_Transform.inverse());
    pose.m_ID = this->m_ID;
    return pose;
}

const Eigen::Vector3f CPose::translation_3x1() const
{
    return m_Transform.block<3, 1>(0, 3);
}

const Eigen::Vector4f CPose::translation_4x1() const
{
    return m_Transform.block<4, 1>(0, 3);
}

const Eigen::Matrix4f CPose::translation_4x4() const
{
    Eigen::Matrix4f trans_mat = Eigen::Matrix4f::Identity();
    trans_mat.block<3, 1>(0, 3) = m_Transform.block<3, 1>(0, 3);
    return trans_mat;
}

const Eigen::Matrix3f CPose::rotation_3x3() const
{
    return m_Transform.block<3, 3>(0, 0);
}

const Eigen::Matrix4f CPose::rotation_4x4() const
{
    Eigen::Matrix4f rot_mat = Eigen::Matrix4f::Identity();
    rot_mat.block<3, 3>(0, 0) = m_Transform.block<3, 3>(0, 0);
    return rot_mat;
}

const Eigen::Vector3f CPose::rotation_euler() const
{
    return R2euler(m_Transform.block<3, 3>(0, 0).matrix());
}

const CQuaternion CPose::rotation_quat() const
{
    const Eigen::Matrix3f R = m_Transform.block<3, 3>(0, 0).matrix();
    return CQuaternion::rotmat2quaternion(R);
}

Eigen::Matrix3f CPose::euler2R_ZYX(const float& _x, const float& _y, const float& _z)
{
    Eigen::Matrix3f rot_x, rot_y, rot_z;

    rot_x << 1.f, 0.f, 0.f,
        0.f, cosf(_x), -sinf(_x),
        0.f, sinf(_x), cosf(_x);

    rot_y << cosf(_y), 0.f, sinf(_y),
        0.f, 1.f, 0.f,
        -sinf(_y), 0.f, cosf(_y);

    rot_z << cosf(_z), -sinf(_z), 0.f,
        sinf(_z), cosf(_z), 0.f,
        0.f, 0.f, 1.f;

    return rot_z * rot_y * rot_x;
}

Eigen::Vector3f CPose::R2euler(const Eigen::Matrix3f& _R)
{
    return Eigen::Vector3f{ 
        atan2f(_R(2, 1), _R(2, 2)),
        atan2f(-_R(2, 0), sqrtf(_R(2, 1) * _R(2, 1) + _R(2, 2) * _R(2, 2))),
        atan2f(_R(1, 0), _R(0, 0))
    };
}

Eigen::Matrix4f CPose::Rt2T(const float& _rot_euler_x, const float& _rot_euler_y, const float& _rot_euler_z,
    const float& _trans_x, const float& _trans_y, const float& _trans_z)
{
    Eigen::Matrix4f T;
        
    T.block<3, 3>(0, 0) = euler2R_ZYX(_rot_euler_x, _rot_euler_y, _rot_euler_z);
    T(0, 3) = _trans_x;
    T(1, 3) = _trans_y;
    T(2, 3) = _trans_z;
    T.row(3) << 0.f, 0.f, 0.f, 1.f;

    return T;
}

Eigen::Matrix4f CPose::Rt2T(const float& _rot_quat_x, const float& _rot_quat_y, const float& _rot_quat_z, const float& _rot_quat_w,
    const float& _trans_x, const float& _trans_y, const float& _trans_z)
{
    Eigen::Matrix4f T;

    T.block<3, 3>(0, 0) = CQuaternion::quaternion2rotmat(Eigen::Vector4f{ _rot_quat_x, _rot_quat_y, _rot_quat_z, _rot_quat_w });
    T(0, 3) = _trans_x;
    T(1, 3) = _trans_y;
    T(2, 3) = _trans_z;
    T.row(3) << 0.f, 0.f, 0.f, 1.f;

    return T;
}

Eigen::Matrix4f CPose::Rt2T(const Eigen::Matrix3f& _R, const Eigen::Vector3f& _t)
{
    Eigen::Matrix4f T;

    // Rotation
    T.block(0, 0, 3, 3) = _R;

    // Translation
    T(0, 3) = _t(0);
    T(1, 3) = _t(1);
    T(2, 3) = _t(2);

    // Last row
    T.row(3) << 0.f, 0.f, 0.f, 1.f;

    return T;
}

float CPose::clampf(const float& _val, const float& _min, const float& _max)
{
    if (_val < _min)
        return _min;
    if (_val > _max)
        return _max;
    return _val;
}
