// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSTATESPACEMODELDRONE_H_
#define CSTATESPACEMODELDRONE_H_

#include "CyC_TYPES.h"
#include <iostream>
#include "CStateSpaceModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"

/*
 * x := [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel]^T
 * y := [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel]^T
 * u := [thrust, roll, pitch, yaw_rate]
 *
 * u is physical throughout the pipeline -- thrust in [N], roll/pitch as angle
 * setpoints in [rad], yaw as a rate setpoint in [rad/s]. Normalisation to a
 * backend's own units (BetaFlight's [0,1] / [-1,1] RC sticks, ANAFI's
 * percentages) belongs to the actuator filter that talks to that backend, so
 * that every other consumer of the control datablock -- simulator, data
 * channel, visualisers -- reads one airframe-independent quantity.
 *
 * To parameterise the system the following physical measurements are required:
 */

#define NO_DRONE_MODEL_STATES 15
#define NO_DRONE_MODEL_INPUTS 4
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
                configFile.lookupValue("Cdx", m_Cdx);
                configFile.lookupValue("Cdy", m_Cdy);
                configFile.lookupValue("Cdz", m_Cdz);

                configFile.lookupValue("thrust_to_weight", m_ThrustToWeight);
                configFile.lookupValue("max_tilt", m_MaxTilt);
                configFile.lookupValue("max_yaw_rate", m_MaxYawRate);

                if (m_ThrustToWeight <= 1.f)
                    spdlog::error("DroneModel: thrust_to_weight = {} (must be > 1.0, or the drone cannot lift its own weight).", m_ThrustToWeight);

                configFile.lookupValue("Kpx", m_Kpx);
                configFile.lookupValue("Kpy", m_Kpy);
                configFile.lookupValue("Kdx", m_Kdx);
                configFile.lookupValue("Kdy", m_Kdy);
                configFile.lookupValue("Kix", m_Kix);
                configFile.lookupValue("Kiy", m_Kiy);

                configFile.lookupValue("Kpz", m_Kpz);
                configFile.lookupValue("Kdz", m_Kdz);
                configFile.lookupValue("Kiz", m_Kiz);

                configFile.lookupValue("Kp_yaw", m_Kp_yaw);
                configFile.lookupValue("Kd_yaw", m_Kd_yaw);
                configFile.lookupValue("Ki_yaw", m_Ki_yaw);

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

        // Thrust that holds the drone in a level hover [N]
        float hoverThrust() const { return m_Mass * GRAVITY; }

        // Thrust of all rotors together at full stick [N]
        float maxThrust() const { return m_Mass * GRAVITY * m_ThrustToWeight; }

    public:
        float m_Mass = 0.2f;
        float m_Ixx = 1.f;
        float m_Iyy = 1.f;
        float m_Izz = 1.f;
        float m_Cdx = 1.f;
        float m_Cdy = 1.f;
        float m_Cdz = 1.f;

        // Actuation envelope: the physical units of u, and what an actuator
        // divides by to reach a backend's normalised stick range.
        float m_ThrustToWeight = 2.f;       // total static thrust / weight [-]
        float m_MaxTilt = 0.5236f;          // max commandable roll/pitch [rad]
        float m_MaxYawRate = 3.1416f;       // max commandable yaw rate [rad/s]

        float m_Kpx = 0.f;
        float m_Kpy = 0.f;

        float m_Kdx = 0.f;
        float m_Kdy = 0.f;

        float m_Kix = 0.f;
        float m_Kiy = 0.f;

        // Altitude Gains (Needs to be higher to fight gravity effectively)
        float m_Kpz = 0.f;
        float m_Kdz = 0.f;
        float m_Kiz = 0.f;

        // Yaw Gains
        float m_Kp_yaw = 0.f;
        float m_Kd_yaw = 0.f;
        float m_Ki_yaw = 0.f;
};

#endif /* CSTATESPACEMODELDRONE_H_ */
