// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSTATESPACEMODELVEHICLE_H_
#define CSTATESPACEMODELVEHICLE_H_

#include "CCR_TYPES.h"
#include <iostream>
#include "CStateSpaceModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"

/*
 * x := [x_position, y_position, velocity, yaw_angle]^T
 * y := [x_position, y_position, yaw_angle]^T
 * u := [acceleration, steering_angle]
 *
 * To parameterise the system the following physical measurements are required:
 * L := lenght of the vehicle [m]
 */

#define NO_VEHICLE_MODEL_STATES 4
#define NO_VEHICLE_MODEL_INPUTS 2
#define NO_VEHICLE_MODEL_OUTPUTS 4

struct VehicleModel : public StateSpaceModel<NO_VEHICLE_MODEL_STATES, NO_VEHICLE_MODEL_INPUTS, NO_VEHICLE_MODEL_OUTPUTS>
{
    public:
        VehicleModel(const float _vehicle_length,
            const float _vehicle_width,
            const float _full_vehicle_length,
            const float _full_vehicle_width,
            const float _wheel_radius,
            const float _max_steering_angle,
            const float _max_steering_rate,
            const float _max_forward_speed,
            const float _max_reverse_speed,
            const float _max_acceleration,
            const CCR_INT _pulses_per_rotation_left,
            const CCR_INT _pulses_per_rotation_right)
        {
            m_fLongDistWheels = _vehicle_length;
            m_fLatDistWheels = _vehicle_width;
            m_fVehicleLength = _full_vehicle_length;
            m_fVehicleWidth = _full_vehicle_width;
            m_fWheelRadius = _wheel_radius;
            m_fWheelCircumference = 2.f * PI * m_fWheelRadius;
            m_fMaxSteeringAngleRad = _max_steering_angle;
            m_fSteeringRateResolution = _max_steering_rate;
            m_fMaxForwardSpeed = _max_forward_speed;
            m_fMaxReverseSpeed = _max_reverse_speed;
            m_fMaxAcceleration = _max_acceleration;
            m_iPulsesPerRotationLeft = _pulses_per_rotation_left;
            m_iPulsesPerRotationRight = _pulses_per_rotation_right;

            init();
        }

        VehicleModel(const std::string& _vehicle_model_file)
        {
            if (fs::exists(_vehicle_model_file.c_str()))
            {
                libconfig::Config configFile;
                configFile.readFile(_vehicle_model_file.c_str());
                
                const libconfig::Setting& rootConfig = configFile.getRoot();
                configFile.lookupValue("type", m_strModelType);
                configFile.lookupValue("long_dist_wheels", m_fLongDistWheels);
                configFile.lookupValue("lat_dist_wheels", m_fLatDistWheels);
                configFile.lookupValue("vehicle_length", m_fVehicleLength);
                configFile.lookupValue("vehicle_width", m_fVehicleWidth);
                configFile.lookupValue("wheel_radius", m_fWheelRadius);
                m_fWheelCircumference = 2.f * PI * m_fWheelRadius;
                configFile.lookupValue("max_steering_angle", m_fMaxSteeringAngleRad);
                configFile.lookupValue("steering_rate_resolution", m_fSteeringRateResolution);
                configFile.lookupValue("max_forward_speed", m_fMaxForwardSpeed);
                configFile.lookupValue("max_reverse_speed", m_fMaxReverseSpeed);
                configFile.lookupValue("max_acceleration", m_fMaxAcceleration);
                configFile.lookupValue("pulses_per_rotation_left", m_iPulsesPerRotationLeft);
                configFile.lookupValue("pulses_per_rotation_right", m_iPulsesPerRotationRight);

                init();
            }
            else
            {
                spdlog::error("{}: Vehicle model file does not exist.", typeid(*this).name());
            }
        }

        void init()
        {
            // Define the system matrix
            A << 1.f, 0.f, 0.f, 0.f,
                0.f, 1.f, 0.f, 0.f,
                0.f, 0.f, 1.f, 0.f,
                0.f, 0.f, 0.f, 1.f;

            // Define the input matrix
            B << 0.f, 0.f,
                0.f, 0.f,
                0.f, 0.f,
                0.f, 0.f;

            // Define the output matrix
            C << 1.f, 0.f, 0.f, 0.f,
                0.f, 1.f, 0.f, 0.f,
                0.f, 0.f, 1.f, 0.f,
                0.f, 0.f, 0.f, 1.f;

            // Define the direct transmission matrix
            D = Eigen::MatrixXf::Zero(4, 2);

            // Define the linearization constant
            //E = Eigen::MatrixXf::Zero(4, 1);

            // Define the state weights Q
            Q = Eigen::MatrixXf::Identity(states, states);

            // Define the control weights R
            R = Eigen::MatrixXf::Identity(inputs, inputs);
        }

        void linearizeMethod1(const float &velocity, const float &yaw_angle, const float &ref_steer_angle, const float &dt)
        {
            // Linearize the system matrix
            A << 1.,    0.,     cos(yaw_angle)*dt,                              -velocity * sin(yaw_angle)*dt,
                 0,     1.,     sin(yaw_angle)*dt,                              velocity*cos(yaw_angle)*dt,
                 0,     0,      1.,                                             0,
                 0,     0,      (tan(ref_steer_angle) / m_fLongDistWheels)*dt,   1.;

            // Linearize the input matrix
            B << 0.,    0.,
                 0.,    0.,
                 dt,    0.,
                 0.,    velocity / (cos(ref_steer_angle) * cos(ref_steer_angle) * m_fLongDistWheels);

            // Linearization constant
            E << velocity * sin(yaw_angle) * yaw_angle * dt,
                -velocity * cos(yaw_angle) * yaw_angle * dt,
                0.,
                -(velocity * ref_steer_angle / (m_fLongDistWheels * cos(ref_steer_angle) * cos(ref_steer_angle))) * dt;
        };

        void linearizeMethod2(const float velocity, const float yaw_angle, const float ref_steer_angle, const float dt)
        {
            const float lr = m_fLongDistWheels / 2.f; // center of gravity at the center of vehicle
            const float lf = m_fLongDistWheels / 2.f; // center of gravity at the center of vehicle

            const float tans = tan(ref_steer_angle);
            const float beta = atanf(tans * lr / m_fLongDistWheels);

            const float common1 = velocity * lr * powf(1.f / cosf(ref_steer_angle), 2.f);
            const float common2 = lr * lr * tans * tans / powf(m_fLongDistWheels, 2.f) + 1.f;

            const float dx_dv = cosf(yaw_angle + beta);
            const float dx_dw = -velocity * sinf(yaw_angle + beta);
            const float dx_du2_num = -common1 * sinf(yaw_angle + beta);
            const float dx_du2_den = m_fLongDistWheels * common2;

            const float dy_dv = sinf(yaw_angle + beta);
            const float dy_dw = velocity * cosf(yaw_angle + beta);
            const float dy_du2_num = common1 * cosf(yaw_angle + beta);
            const float dy_du2_den = dx_du2_den; // same denominator

            const float dw_dv = tans / (m_fLongDistWheels * sqrtf(common2));
            const float dw_du2_num = velocity * powf(1.f / cos(ref_steer_angle), 2.f);
            const float dw_du2_den = m_fLongDistWheels * sqrtf(powf(common2, 3.f));

            // Linearize the system matrix
            A << 0.f, 0.f, dx_dv, dx_dw,
                 0.f, 0.f, dy_dv, dy_dw,
                 0.f, 0.f, 0.f,   0.f,
                 0.f, 0.f, dw_dv, 0.f;

            // Linearize the input matrix
            B << 0.f, dx_du2_num / dx_du2_den,
                 0.f, dy_du2_num / dy_du2_den,
                 1.f, 0.f,
                 0.f, dw_du2_num / dw_du2_den;

//            // Linearization constant
//            E << velocity * sin(yaw_angle) * yaw_angle * dt,
//                -velocity * cos(yaw_angle) * yaw_angle * dt,
//                0.,
//                -(velocity * ref_steer_angle / (m_fVehicleLength * cos(ref_steer_angle) * cos(ref_steer_angle))) * dt;
        };

    public:
        std::string m_strModelType;                     // Type of the model (ackermann, differential, etc.)
        float       m_fLongDistWheels = -1.f;           // Distance between front and rear axles [m]
        float       m_fLatDistWheels = -1.f;            // Distance between lateral axles        [m]
        float       m_fVehicleLength = -1.f;            // Lenght of the vehicle    [m]
        float       m_fVehicleWidth = -1.f;             // Width of the vehicle     [m]
        float       m_fWheelRadius = -1.f;              // Wheel radius             [m]
        float       m_fWheelCircumference = -1.f;       // Wheel circumference      [m]     (2*PI*radius)
        float       m_fMaxSteeringAngleRad = -1.f;      // Maximum steering angle   [rad]
        float       m_fSteeringRateResolution = -1.f;   // Maximum steering speed   [rad/s]
        float       m_fMaxForwardSpeed = -1.f;          // Maximum speed            [m/s]
        float       m_fMaxReverseSpeed = -1.f;          // Minimum speed            [m/s]
        float       m_fMaxAcceleration = -1.f;          // Maximum acceleration     [m/ss]
        CCR_INT     m_iPulsesPerRotationLeft = 1;       // Pulses generated by a full rotation
        CCR_INT     m_iPulsesPerRotationRight = 1;      // Pulses generated by a full rotation
};

#endif /* CSTATESPACEMODELVEHICLE_H_ */
