// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSimulationArm_H_
#define CSimulationArm_H_

#include "CyC_TYPES.h"
#include "CStateSpaceModelArm.h"

class CSimulationArm
{
public:
    CSimulationArm(CArmModel& _model) :
        model(_model)
    {
        // Initial state
        x = Eigen::VectorXf::Zero(model.getDH()->size() * NO_ARM_LINK_MODEL_STATES);
    }

    float angular_diff(float theta1, float theta2)
    {
        return std::fmodf((theta1 - theta2 + PI), (2.f * PI)) - PI;
    }

    void step(Eigen::VectorXf& u, const float& dt)
    {
        float Kp = 2.f;
        std::vector<CArmModel::LinkDH>* DHArm = model.getDH();
        
        // Assert sizes of control input and number of links
        if (u.size() != DHArm->size())
        {
            spdlog::info("CSimulationArm::step: error: number of control inputs must equal number of arm links");
            return;
        }

        // Update joint angles
        for (size_t i = 0; i < DHArm->size(); ++i)
        {
            // joint_angles = joint_angles + Kp * ang_diff(joint_goal_angles, joint_angles) * dt
            DHArm->at(i).theta += Kp * angular_diff(u[i], DHArm->at(i).theta) * dt;
            x[i * NO_ARM_LINK_MODEL_STATES] = DHArm->at(i).theta;
            x[i * NO_ARM_LINK_MODEL_STATES + 1] = DHArm->at(i).alpha;
            x[i * NO_ARM_LINK_MODEL_STATES + 2] = DHArm->at(i).a;
            x[i * NO_ARM_LINK_MODEL_STATES + 3] = DHArm->at(i).d;
        }
    }

public:
    CArmModel& model;
    Eigen::VectorXf x;
};

#endif /* CSimulationArm_H_ */
