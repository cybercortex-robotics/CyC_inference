// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef ArmModel_H_
#define ArmModel_H_

#include "CCR_TYPES.h"
#include <iostream>
#include "CStateSpaceModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"

 /*
  * x := [dh_0, ..., dh_n]^T
  * u := [desired_theta_0, ..., desired_theta_n]^T  -> desired joint angles
  */

#define NO_ARM_LINK_MODEL_STATES 4

class CArmModel
{
public:
    struct LinkDH {
        LinkDH()
        {}

        LinkDH(float _theta, float _alpha, float _a, float _d) :
            theta(_theta), alpha(_alpha), a(_a), d(_d)
        {}

        static const Eigen::Matrix4f transform(float _theta, float _alpha, float _a, float _d)
        {
            const float sin_theta = sinf(_theta);
            const float cos_theta = cosf(_theta);
            const float sin_alpha = sinf(_alpha);
            const float cos_alpha = cosf(_alpha);

            Eigen::Matrix4f trans;
            trans << cos_theta, -sin_theta * cos_alpha, sin_theta* sin_alpha, _a * cos_theta,
                sin_theta, cos_theta* cos_alpha, -cos_theta * sin_alpha, _a * sin_theta,
                0.f, sin_alpha, cos_alpha, _d,
                0.f, 0.f, 0.f, 1.f;

            return trans;
        }

        const Eigen::Matrix4f transform() const
        {
            return transform(theta, alpha, a, d);
        }

        const CPose pose() const
        {
            return CPose(transform());
        }

        float theta;
        float alpha;
        float a;
        float d;
    };

public:
    CArmModel()
    {
        init();
    }

    CArmModel(const std::string& _arm_model_file)
    {
        if (fs::exists(_arm_model_file.c_str()))
        {
            libconfig::Config configFile;
            
            configFile.readFile(_arm_model_file.c_str());
            
            const libconfig::Setting& rootConfig = configFile.getRoot();
            const libconfig::Setting& Links = rootConfig["Links"];
            
            for (CCR_INT i = 0; i < Links.getLength(); ++i)
            {
                LinkDH link;
                const libconfig::Setting& LinksConfig = Links[i];
                LinksConfig.lookupValue("theta", link.theta);
                LinksConfig.lookupValue("alpha", link.alpha);
                LinksConfig.lookupValue("a", link.a);
                LinksConfig.lookupValue("d", link.d);
                m_DHArmModel.push_back(link);
            }

            init();
        }
        else
        {
            spdlog::error("ArmModel: Drone model file does not exist.");
        }
    }

    void init()
    {}

    std::vector<CArmModel::LinkDH>* getDH() { return &m_DHArmModel; };

    const std::vector<CPose> getKinematicChain() const
    {
        std::vector<CPose> kinematic_chain;
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

        for (int i = 0; i < m_DHArmModel.size(); ++i)
        {
            trans = trans * m_DHArmModel[i].transform();
            kinematic_chain.emplace_back(CPose(trans));
        }

        return kinematic_chain;
    }

    static const void state2kinematicchain(const Eigen::VectorXf& state, std::vector<CPose>& kinematic_chain)
    {
        kinematic_chain.clear();
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

        for (CCR_INT i = 0; i < (state.size() / NO_ARM_LINK_MODEL_STATES); ++i)
        {
            float theta = state[i * NO_ARM_LINK_MODEL_STATES];
            float alpha = state[i * NO_ARM_LINK_MODEL_STATES + 1];
            float a = state[i * NO_ARM_LINK_MODEL_STATES + 2];
            float d = state[i * NO_ARM_LINK_MODEL_STATES + 3];

            trans = trans * LinkDH::transform(theta, alpha, a, d);
            kinematic_chain.push_back(CPose(trans));
        }
    }

private:
    std::vector<CArmModel::LinkDH> m_DHArmModel;
};

#endif /* ArmModel_H_ */
