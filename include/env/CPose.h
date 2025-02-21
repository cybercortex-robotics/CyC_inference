// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/*
 * https://www.andre-gaschler.com/rotationconverter/
 * https://github.com/gaschler/rotationconverter
 * https://github.com/mrdoob/three.js
 * http://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
 * https://math.stackexchange.com/questions/1972196/determining-the-unique-set-of-euler-angles-that-produce-a-set-of-3-left-hand
 * http://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm
 * https://stackoverflow.com/questions/22157435/difference-between-the-two-quaternions
 */

#ifndef CPose_H_
#define CPose_H_

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include "env/CQuaternion.h"
#include <iostream>
#include <iomanip>
#include "os/CStringUtils.h"

struct CPose
{
public:
    CPose(int _id = -1);
    
    CPose(const Eigen::Matrix4f& _pose, int _id = -1);

    CPose(const float& _trans_x, const float& _trans_y, const float& _trans_z,
        const float& _rot_x, const float& _rot_y, const float& _rot_z, int _id = -1);

    CPose(const float& _trans_x, const float& _trans_y, const float& _trans_z,
        const float& _rot_quat_x, const float& _rot_quat_y, const float& _rot_quat_z, const float& _rot_quat_w, int _id = -1);

    CPose(const Eigen::Vector3f& _trans, const Eigen::Vector3f& _rot_euler, int _id = -1);

    CPose(const Eigen::Vector3f& _trans, const Eigen::Vector4f& _rot_quat, int _id = -1);

    CPose(const Eigen::Vector3f& _trans, const Eigen::Matrix3f& _rot_mat, int _id = -1);

    CPose(const std::string& _str, int _id = -1);

    const int getID() const;
    const void setID(int _id);

    CPose operator*(const CPose& _pose) const;

    CPose operator+(const CPose& _q) const;

    CPose operator-(const CPose& _q) const;

    void update(const Eigen::Matrix4f& _pose);

    void update(const float& _trans_x, const float& _trans_y, const float& _trans_z,
        const float& _rot_euler_x, const float& _rot_euler_y, const float& _rot_euler_z);

    void update(const float& _trans_x, const float& _trans_y, const float& _trans_z,
        const float& _rot_quat_x, const float& _rot_quat_y, const float& _rot_quat_z, const float& _rot_quat_w);

    void update(const Eigen::Vector3f& _trans, const Eigen::Vector3f& _rot_euler);

    void update(const Eigen::Vector3f& _trans, const Eigen::Vector4f& _rot_quat);

    void update(const Eigen::Vector3f& _trans, const Eigen::Matrix3f& _rot_mat);

    void update(const std::string& _str);

    void update_R(const Eigen::Matrix3f& _R);

    void update_R(const Eigen::Vector3f& _rot_euler);

    void update_R(const Eigen::Vector4f& _rot_quat);

    void update_R(const float& _rot_euler_x, const float& _rot_euler_y, const float& _rot_euler_z);

    void update_R(const float& _rot_quat_x, const float& _rot_quat_y, const float& _rot_quat_z, const float& _rot_quat_w);

    void update_t(const Eigen::Vector3f& _trans);

    void update_t(const float& _trans_x, const float& _trans_y, const float& _trans_z);

    const Eigen::Matrix4f transform() const;

    const CPose inverse() const;

    const Eigen::Vector3f translation_3x1() const;

    const Eigen::Vector4f translation_4x1() const;

    const Eigen::Matrix4f translation_4x4() const;

    const Eigen::Matrix3f rotation_3x3() const;

    const Eigen::Matrix4f rotation_4x4() const;

    const Eigen::Vector3f rotation_euler() const;

    const CQuaternion rotation_quat() const;

    /**
     * \brief       Creates a rotation matrix from 3 euler angles, given the Z, Y, X multimplication order
     *
     * \param _x    Input rotation around the X axis
     * \param _y    Input rotation around the Y axis
     * \param _z    Input rotation around the Z axis
     * \return      3x3 rotation matrix
     **/
    static Eigen::Matrix3f euler2R_ZYX(const float& _x, const float& _y, const float& _z);

    /**
     * \brief       Calculates the euler angles from a rotation matrix
     *              There is no unique solution. The function always returns a positive y angle
     *
     * \param _x    Input rotation matrix
     * \return      x, y, z euler angles
     **/
    static Eigen::Vector3f R2euler(const Eigen::Matrix3f& _R);
    
    static Eigen::Matrix4f Rt2T(const float& _rot_euler_x, const float& _rot_euler_y, const float& _rot_euler_z,
        const float& _trans_x, const float& _trans_y, const float& _trans_z);

    static Eigen::Matrix4f Rt2T(const float& _rot_quat_x, const float& _rot_quat_y, const float& _rot_quat_z, const float& _rot_quat_w,
        const float& _trans_x, const float& _trans_y, const float& _trans_z);

    /**
     * \brief           Assembles a transformation matrix from a 3x3 rotation matrix and a 3x1 translation vector
     *
     * \param _R        Input rotation matrix
     * \param _t        Input translation vector
     * \return          Transformation matrix T
     **/
    static Eigen::Matrix4f Rt2T(const Eigen::Matrix3f& _R, const Eigen::Vector3f& _t);

    friend auto operator<<(std::ostream& _os, CPose const& _pose) -> std::ostream&
    {
        Eigen::Vector3f euler = _pose.rotation_euler() * 57.295779f;
        Eigen::Vector3f trans = _pose.translation_3x1();

        return _os << _pose.m_ID << ": " << "R: [" << euler.x() << ", " << euler.y() << ", " << euler.z() <<
            "] trans: [" << trans.x() << ", " << trans.y() << ", " << trans.z() << "]";
    }

private:
    static float clampf(const float& _val, const float& _min, const float& _max);

private:
    Eigen::Matrix4f m_Transform = Eigen::Matrix4f::Identity();

    // Pose ID used for landmark recognition
    int m_ID = -1;
};

#endif /* CPose_H_ */
