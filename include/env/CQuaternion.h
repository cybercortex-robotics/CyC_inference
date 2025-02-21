// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/*
 * http://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm
 * https://stackoverflow.com/questions/22157435/difference-between-the-two-quaternions
 */

#ifndef CQuaternion_H_
#define CQuaternion_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>

struct CQuaternion
{
public:
    CQuaternion();

    CQuaternion(const Eigen::Vector4f _q);

    CQuaternion(const float _x, const float _y, const float _z, const float _w);

    CQuaternion(const Eigen::Matrix3f _R);

    CQuaternion(const float _roll, const float _pitch, const float _yaw);

    void update(const Eigen::Vector4f _q);

    void update(const float _x, const float _y, const float _z, const float _w);

    void update(const Eigen::Matrix3f _R);

    void update(const float _roll, const float _pitch, const float _yaw);

    const CQuaternion operator*(const CQuaternion& _q) const;

    const CQuaternion operator/ (float _s) const;

    CQuaternion operator+(const CQuaternion& _q) const;

    CQuaternion operator-(const CQuaternion& _q) const;

    static float dot(const CQuaternion& q1, const CQuaternion& q2);

    CQuaternion inverse() const;

    CQuaternion conjugate() const;

    Eigen::Matrix3f to_rotmat() const;

    Eigen::Vector3f to_euler_ZYX() const;

    Eigen::Vector4f to_vector() const;

    const float x() const;
    const float y() const;
    const float z() const;
    const float w() const;

    /**
     * \brief       Calculates the quaternion representation of a rotation matrix
     *
     * \param _x    Input rotation matrix
     * \return      Quaternion in the sequence (x, y, z, w), where [x, y, z] is the vector and w is the scalar
     **/
    static CQuaternion rotmat2quaternion(const Eigen::Matrix3f _R);

    /**
     * \brief       Calculates the rotation matrix from a quaternion
     *
     * \param _x    Input quaternion in the sequence (x, y, z, w), where [x, y, z] is the vector and w is the scalar
     * \return      Rotation matrix
     **/
    static Eigen::Matrix3f quaternion2rotmat(const float _x, const float _y, const float _z, const float _w);
    static Eigen::Matrix3f quaternion2rotmat(const Eigen::Vector4f _q);

    /**
     * \brief       Calculates the euler angles from a quaternion
     *
     * \param _x    Input quaternion in the sequence (x, y, z, w), where [x, y, z] is the vector and w is the scalar
     * \return      Rotation matrix
     **/
    static CQuaternion euler2quaternion(const float _roll, const float _pitch, const float _yaw);

    /**
     * \brief       Calculates the euler angles from a quaternion
     *              There is no unique solution. Care must be taken.
     *
     * \param _x    Input quaternion
     * \return      x, y, z euler angles
     **/
    static Eigen::Vector3f quaternion2euler_ZYX(const float _x, const float _y, const float _z, const float _w);
    static Eigen::Vector3f quaternion2euler_ZYX(const Eigen::Vector4f _q);

    friend auto operator<<(std::ostream& _os, CQuaternion const& _q) -> std::ostream&
    {
        Eigen::Vector3f euler = _q.to_euler_ZYX();
        return _os << euler.x() << "\t" << euler.y() << "\t" << euler.z() << "\t(" << _q.m_x << "\t" << _q.m_y << "\t" << _q.m_z << "\t" << _q.m_w << ")";
    }

private:
    float m_x;
    float m_y;
    float m_z;
    float m_w;
};

#endif /* CQuaternion_H_ */
