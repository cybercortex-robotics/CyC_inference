#include "CModelLeggedDynamics.h"
#include <Eigen/Dense>

float pi = 3.14159265359f;

Eigen::Matrix3f inertia_r(const Eigen::Vector3f& q_v)
{
    float q2 = q_v(1);
    float q3 = q_v(2);
    float t2 = cos(q2);
    float t3 = cos(q3);
    float t4 = sin(q2);
    float t5 = sin(q3);
    float t6 = q2 * 2.0f;
    float t7 = cos(t6);
    float t8 = sin(t6);

    float I1 = (t3 * 6.64e-3f + t7 * 2.0057112e-2f - t8 * 4.825e-6f + t3 * t7 * 6.64e-3f - t5 * t8 * 6.64e-3f + t3 * t3 * t7 * 6.65605e-3f) + (t3 * t5 * t8 * (-6.65605e-3f) + 4.432714576e-2f);
    float I2 = t2 * 2.2448e-5f - t4 * 2.0103909e-2f - t2 * t5 * 2.640997e-3f - t3 * t4 * 2.640997e-3f;
    float I3 = sin(q2 + q3) * (-2.640997e-3f);
    float I4 = t2 * 2.2448e-5f - t4 * 2.0103909e-2f - t3 * t4 * 2.640997e-3f - t2 * sin(q3) * 2.640997e-3f;
    float I5 = t3 * 1.328e-2f + 5.5200214e-2f;
    float I6 = t3 * 6.64e-3f + 6.672426e-3f;
    float I7 = sin(q2 + q3) * (-2.640997e-3f);
    float I8 = cos(q3) * 6.64e-3f + 6.672426e-3f;
    float I9 = 6.672426e-3f;

    Eigen::Matrix3f I;

    I << I1, I2, I3,
        I4, I5, I6,
        I7, I8, I9;

    return I;
}

Eigen::Matrix3f coriolis_r(const Eigen::Vector3f& q, const Eigen::Vector3f& qd)
{
    float q2 = q(1);
    float q3 = q(2);
    float qd1 = qd(0);
    float qd2 = qd(1);
    float qd3 = qd(2);

    float t2 = sin(q3);
    float t22 = cos(q3);
    float t3 = q2 + q3;
    float t4 = q2 * 2.0f;
    float t5 = q3 * 2.0f;
    float t6 = cos(t4);
    float t7 = sin(t4);
    float t8 = cos(t3);
    float t9 = q2 + t3;
    float t11 = t3 * 2.0f;
    float t10 = sin(t9);
    float t12 = sin(t11);
    float t15 = qd2 * t8 * 2.640997e-3f;
    float t16 = qd3 * t8 * 2.640997e-3f;
    float t13 = qd1 * t12 * 3.328025e-3f;
    float t17 = -t15;
    float t18 = -t16;
    float t14 = -t13;

    float c1 = qd3 * t2 * (-3.32e-3f) - qd2 * t6 * 4.825e-6f - qd2 * t7 * 2.3385137e-2f - qd2 * t10 * 6.64e-3f - qd3 * t10 * 3.32e-3f - qd2 * t12 * 3.328025e-3f + qd3 * t12 * (-3.328025e-3f);
    float c2 = t14 + t17 + t18 - qd1 * t6 * 4.825e-6f - qd1 * t7 * 2.3385137e-2f - qd1 * t10 * 6.64e-3f - qd2 * cos(q2) * 2.0103909e-2f - qd2 * sin(q2) * 2.2448e-5f;
    float c3 = t14 + t17 + t18 - qd1 * t2 * 3.32e-3f - qd1 * t10 * 3.32e-3f;

    float c4 = qd1 * (t6 * 4.825e-6f + t7 * 2.0057112e-2f + t22 * t7 * 6.64e-3f + t2 * t6 * 6.64e-3f + t22 * t22 * t7 * 6.65605e-3f + t22 * t2 * t6 * 6.65605e-3f);
    float c5 = qd3 * t2 * (-6.64e-3f);
    float c6 = t2 * (qd2 + qd3) * (-6.64e-3f);

    float c7 = qd1 * (t2 * 3.32e-3f + sin(q3 * 2.0f + t4) * 3.328025e-3f + sin(q3 + t4) * 3.32e-3f);
    float c8 = qd2 * t2 * 6.64e-3f;
    float c9 = 0.0f;

    Eigen::Matrix3f C;

    C << c1, c2, c3,
        c4, c5, c6,
        c7, c8, c9;

    return C;
}

Eigen::Vector3f gravload_r(const Eigen::Vector3f& q)
{
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    float t2 = cos(q1);
    float t3 = cos(q2);
    float t4 = sin(q1);
    float t5 = sin(q2);
    float t6 = q2 + q3;
    float t7 = sin(t6);

    float G1 = t2 * (-9.69229962e-1f) + t3 * t4 * 2.313198f + t3 * t4 * cos(q3) * 3.25692e-1f - t4 * t5 * sin(q3) * 3.25692e-1f;
    float G2 = t2 * (t5 * 1.179e+3f + t7 * 1.66e+2f) * 1.962e-3f;
    float G3 = t2 * t7 * 3.25692e-1f;

    Eigen::Vector3f G;

    G << G1, G2, G3;

    return G;
}

Eigen::Vector3f Iqdd_r(const Eigen::Vector3f& q, const Eigen::Vector3f& qd, const Eigen::Vector3f& tau)
{
    float Q1 = tau(0);
    float Q2 = tau(1);
    float Q3 = tau(2);
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    float qd1 = qd(0);
    float qd2 = qd(1);
    float qd3 = qd(2);


    float t2 = cos(q1);
    float t3 = cos(q2);
    float t4 = cos(q3);
    float t5 = sin(q1);
    float t6 = sin(q2);
    float t7 = sin(q3);
    float t8 = q2 + q3;
    float t9 = q2 * 2.0f;
    float t10 = q3 * 2.0f;
    float t11 = qd1 * qd1;
    float t12 = cos(t9);
    float t13 = sin(t9);
    float t14 = cos(t8);
    float t15 = sin(t8);
    float t16 = q2 + t8;
    float t18 = t8 * 2.0f;
    float t17 = sin(t16);
    float t19 = sin(t18);
    float t21 = qd2 * t14 * 2.640997e-3f;
    float t22 = qd3 * t14 * 2.640997e-3f;
    float t20 = qd1 * t19 * 3.328025e-3f;

    float et1 = qd3 * t7 * 3.32e-3f + qd2 * t12 * 4.825e-6f + qd2 * t13 * 2.3385137e-2f + qd2 * t17 * 6.64e-3f + qd3 * t17 * 3.32e-3f + qd2 * t19 * 3.328025e-3f;
    float et2 = qd3 * t19 * 3.328025e-3f;
    float et3 = Q1 + t2 * 9.69229962e-1f + qd3 * (t20 + t21 + t22 + qd1 * t7 * 3.32e-3f + qd1 * t17 * 3.32e-3f) - t3 * t5 * 2.313198f;
    float et4 = qd2 * (t20 + t21 + t22 + qd2 * t3 * 2.0103909e-2f + qd2 * t6 * 2.2448e-5f + qd1 * t12 * 4.825e-6f + qd1 * t13 * 2.3385137e-2f + qd1 * t17 * 6.64e-3f) + qd1 * (et1 + et2);
    float et5 = t3 * t4 * t5 * (-3.25692e-1f) + t5 * t6 * t7 * 3.25692e-1f;
    float et6 = t12 * 4.825e-6f + t13 * 2.0057112e-2f + t4 * t13 * 6.64e-3f + t7 * t12 * 6.64e-3f + t4 * t4 * t13 * 6.65605e-3f;
    float et7 = t4 * t7 * t12 * 6.65605e-3f;

    Eigen::Vector3f Iqdd;

    float Iqdd1 = et3 + et4 + et5;
    float Iqdd2 = Q2 - t11 * (et6 + et7) - t2 * (t6 * 1.179e+3f + t15 * 1.66e+2f) * 1.962e-3f + qd3 * t7 * (qd2 + qd3) * 6.64e-3f + qd2 * qd3 * t7 * 6.64e-3f;
    float Iqdd3 = Q3 - t2 * t15 * 3.25692e-1f - t11 * (t7 * 3.32e-3f + t17 * 3.32e-3f + t19 * 3.328025e-3f) - qd2 * qd2 * t7 * 6.64e-3f;

    Iqdd << Iqdd1, Iqdd2, Iqdd3;

    return Iqdd;
}

Eigen::Matrix3f inertia_l(const Eigen::Vector3f& q_v)
{
    float q2 = q_v(1);
    float q3 = q_v(2);
    float t2 = cos(q2);
    float t3 = cos(q3);
    float t4 = sin(q2);
    float t5 = sin(q3);
    float t6 = q2 * 2.0f;
    float t7 = cos(t6);
    float t8 = sin(t6);

    float I1 = t3 * 6.64e-3f + t7 * 2.0057112e-2f + t8 * 4.825e-6f + t3 * t7 * 6.64e-3f - t5 * t8 * 6.64e-3f + t3 * t3 * t7 * 6.65605e-3f + t3 * t5 * t8 * (-6.65605e-3f) + 4.432714576e-2f;
    float I2 = t2 * (-2.2448e-5f) + t4 * 1.9416171e-2f + t2 * t5 * 2.923323e-3f + t3 * t4 * 2.923323e-3f;
    float I3 = sin(q2 + q3) * 2.923323e-3f;
    float I4 = t2 * (-2.2448e-5f) + t4 * 1.9416171e-2f + t3 * t4 * 2.923323e-3f + t2 * sin(q3) * 2.923323e-3f;
    float I5 = t3 * 1.328e-2f + 5.5200214e-2f;
    float I6 = t3 * 6.64e-3f + 6.672426e-3f;
    float I7 = sin(q2 + q3) * 2.923323e-3f;
    float I8 = cos(q3) * 6.64e-3f + 6.672426e-3f;
    float I9 = 6.672426e-3f;

    Eigen::Matrix3f I;

    I << I1, I2, I3,
        I4, I5, I6,
        I7, I8, I9;

    return I;
}

Eigen::Matrix3f coriolis_l(const Eigen::Vector3f& q, const Eigen::Vector3f& qd)
{
    float q2 = q(1);
    float q3 = q(2);
    float qd1 = qd(0);
    float qd2 = qd(1);
    float qd3 = qd(2);

    float t2 = sin(q3);
    float t22 = cos(q3);
    float t3 = q2 + q3;
    float t4 = q2 * 2.0f;
    float t5 = q3 * 2.0f;
    float t6 = cos(t4);
    float t7 = sin(t4);
    float t8 = cos(t3);
    float t9 = q2 + t3;
    float t11 = t3 * 2.0f;
    float t10 = sin(t9);
    float t12 = sin(t11);
    float t15 = qd2 * t8 * 2.923323e-3f;
    float t16 = qd3 * t8 * 2.923323e-3f;
    float t13 = qd1 * t12 * 3.328025e-3f;
    float t17 = -t15;
    float t18 = -t16;
    float t14 = -t13;

    float c1 = qd3 * t2 * (-3.32e-3f) + qd2 * t6 * 4.825e-6f - qd2 * t7 * 2.3385137e-2f - qd2 * t10 * 6.64e-3f - qd3 * t10 * 3.32e-3f - qd2 * t12 * 3.328025e-3f + qd3 * t12 * (-3.328025e-3f);
    float c2 = t14 + t15 + t16 + qd1 * t6 * 4.825e-6f - qd1 * t7 * 2.3385137e-2f - qd1 * t10 * 6.64e-3f + qd2 * cos(q2) * 1.9416171e-2f + qd2 * sin(q2) * 2.2448e-5f;
    float c3 = t14 + t15 + t16 - qd1 * t2 * 3.32e-3f - qd1 * t10 * 3.32e-3f;

    float c4 = qd1 * (t6 * 4.825e-6f + t7 * 2.0057112e-2f + t22 * t7 * 6.64e-3f + t2 * t6 * 6.64e-3f + t22 * t22 * t7 * 6.65605e-3f + t22 * t2 * t6 * 6.65605e-3f);
    float c5 = qd3 * t2 * (-6.64e-3f);
    float c6 = t2 * (qd2 + qd3) * (-6.64e-3f);

    float c7 = qd1 * (t2 * 3.32e-3f + sin(q3 * 2.0f + t4) * 3.328025e-3f + sin(q3 + t4) * 3.32e-3f);
    float c8 = qd2 * t2 * 6.64e-3f;
    float c9 = 0.0f;

    Eigen::Matrix3f C;

    C << c1, c2, c3,
        c4, c5, c6,
        c7, c8, c9;

    return C;
}

Eigen::Vector3f gravload_l(const Eigen::Vector3f& q)
{
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    float t2 = cos(q1);
    float t3 = cos(q2);
    float t4 = sin(q1);
    float t5 = sin(q2);
    float t6 = q2 + q3;
    float t7 = sin(t6);

    float G1 = t2 * 9.69229962e-1f + t3 * t4 * 2.313198f + t3 * t4 * cos(q3) * 3.25692e-1f - t4 * t5 * sin(q3) * 3.25692e-1f;
    float G2 = t2 * (t5 * 1.179e+3f + t7 * 1.66e+2f) * 1.962e-3f;
    float G3 = t2 * t7 * 3.25692e-1f;

    Eigen::Vector3f G;

    G << G1, G2, G3;

    return G;
}

Eigen::Vector3f Iqdd_l(const Eigen::Vector3f& q, const Eigen::Vector3f& qd, const Eigen::Vector3f& tau)
{
    float Q1 = tau(0);
    float Q2 = tau(1);
    float Q3 = tau(2);
    float q1 = q(0);
    float q2 = q(1);
    float q3 = q(2);
    float qd1 = qd(0);
    float qd2 = qd(1);
    float qd3 = qd(2);


    float t2 = cos(q1);
    float t3 = cos(q2);
    float t4 = cos(q3);
    float t5 = sin(q1);
    float t6 = sin(q2);
    float t7 = sin(q3);
    float t8 = q2 + q3;
    float t9 = q2 * 2.0f;
    float t10 = q3 * 2.0f;
    float t11 = qd1 * qd1;
    float t12 = cos(t9);
    float t13 = sin(t9);
    float t14 = cos(t8);
    float t15 = sin(t8);
    float t16 = q2 + t8;
    float t18 = t8 * 2.0f;
    float t17 = sin(t16);
    float t19 = sin(t18);
    float t21 = qd2 * t14 * 2.923323e-3f;
    float t22 = qd3 * t14 * 2.923323e-3f;
    float t20 = qd1 * t19 * 3.328025e-3f;

    float et1 = qd2 * (-t20 + t21 + t22 + qd2 * t3 * 1.9416171e-2f + qd2 * t6 * 2.2448e-5f + qd1 * t12 * 4.825e-6f - qd1 * t13 * 2.3385137e-2f - qd1 * t17 * 6.64e-3f);
    float et2 = -1.0f;
    float et3 = qd3 * t7 * 3.32e-3f - qd2 * t12 * 4.825e-6f + qd2 * t13 * 2.3385137e-2f + qd2 * t17 * 6.64e-3f + qd3 * t17 * 3.32e-3f + qd2 * t19 * 3.328025e-3f;
    float et4 = qd3 * t19 * 3.328025e-3f;
    float et5 = t12 * (-4.825e-6f) + t13 * 2.0057112e-2f + t4 * t13 * 6.64e-3f + t7 * t12 * 6.64e-3f + t4 * t4 * t13 * 6.65605e-3f;
    float et6 = t4 * t7 * t12 * 6.65605e-3f;

    Eigen::Vector3f Iqdd;

    float Iqdd1 = Q1 - t2 * 9.69229962e-1f - t3 * t5 * 2.313198f + et1 * et2 + qd3 * (t20 - t21 - t22 + qd1 * t7 * 3.32e-3f + qd1 * t17 * 3.32e-3f) + qd1 * (et3 + et4) - t3 * t4 * t5 * 3.25692e-1f + t5 * t6 * t7 * 3.25692e-1f;
    float Iqdd2 = Q2 - t11 * (et5 + et6) - t2 * (t6 * 1.179e+3f + t15 * 1.66e+2f) * 1.962e-3f + qd3 * t7 * (qd2 + qd3) * 6.64e-3f + qd2 * qd3 * t7 * 6.64e-3f;
    float Iqdd3 = Q3 - t2 * t15 * 3.25692e-1f - t11 * (t7 * 3.32e-3f + t17 * 3.32e-3f + t19 * 3.328025e-3f) - qd2 * qd2 * t7 * 6.64e-3f;

    Iqdd << Iqdd1, Iqdd2, Iqdd3;

    return Iqdd;
}

// Note: Currently hardcoded for the Unitree A1
Eigen::Vector3f forward_dynamics(const Eigen::Vector3f& q, const Eigen::Vector3f& qd, const Eigen::Vector3f& tau, const bool is_left)
{
    // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    return is_left
        ? inertia_l(q).colPivHouseholderQr().solve(Iqdd_l(q, qd, tau))
        : inertia_r(q).colPivHouseholderQr().solve(Iqdd_r(q, qd, tau));
}

// Note: Currently hardcoded for the Unitree A1
Eigen::Vector3f inverse_dynamics(const Eigen::Vector3f& q, const Eigen::Vector3f& dq, const Eigen::Vector3f& ddq, const bool is_left)
{
    return is_left
        ? (inertia_l(q) * ddq + coriolis_l(q, dq) * dq + gravload_l(q))
        : (inertia_r(q) * ddq + coriolis_r(q, dq) * dq + gravload_r(q));
}

