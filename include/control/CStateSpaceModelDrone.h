// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSTATESPACEMODELDRONE_H_
#define CSTATESPACEMODELDRONE_H_

#include "CCR_TYPES.h"
#include <iostream>
#include "CStateSpaceModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"

/*
 * x := [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel]^T
 * y := [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel]^T
 * u := [thrust, roll_torque, pitch_torque, yaw_torque, vel_x, vel_y, vel_z, angular_vel_yaw]
 *
 * To parameterise the system the following physical measurements are required:
 */

#define NO_DRONE_MODEL_STATES 15
#define NO_DRONE_MODEL_INPUTS 8
#define NO_DRONE_MODEL_OUTPUTS 15

struct DroneModel : public StateSpaceModel<NO_DRONE_MODEL_STATES, NO_DRONE_MODEL_INPUTS, NO_DRONE_MODEL_OUTPUTS>
{
    public:
        DroneModel()
        {
            init();
        }

        DroneModel(const std::string& _drone_model_file)
        {
            if (fs::exists(_drone_model_file.c_str()))
            {
                libconfig::Config configFile;
                configFile.readFile(_drone_model_file.c_str());
                
                const libconfig::Setting& rootConfig = configFile.getRoot();
                configFile.lookupValue("mass", m_Mass);
                configFile.lookupValue("Ixx", m_Ixx);
                configFile.lookupValue("Iyy", m_Iyy);
                configFile.lookupValue("Izz", m_Izz);

                init();
            }
            else
            {
                spdlog::error("DroneModel: Drone model file does not exist.");
            }
        }

        void init()
        {
            // Define the system matrix
            A = Eigen::MatrixXf::Identity(NO_DRONE_MODEL_STATES, NO_DRONE_MODEL_STATES);

            // Define the output matrix
            C << Eigen::MatrixXf::Identity(NO_DRONE_MODEL_STATES, NO_DRONE_MODEL_STATES);
        }

    public:
        float m_Mass = 0.2f;
        float m_Ixx = 1.f;
        float m_Iyy = 1.f;
        float m_Izz = 1.f;
};

#endif /* CSTATESPACEMODELDRONE_H_ */
