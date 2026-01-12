/*
 * CDenavitHartenberg.cpp
 *
 *  Created on: 08.10.2022
 *      Author: Sorin Grigorescu
 */

#include "CDenavitHartenberg.h"

CLinkDH::CLinkDH()
{}

CLinkDH::CLinkDH(float _theta, float _alpha, float _a, float _d) :
    theta(_theta), alpha(_alpha), a(_a), d(_d)
{}

const Eigen::Matrix4f CLinkDH::transform(float _theta, float _alpha, float _a, float _d)
{
    const float sin_theta = sinf(_theta);
    const float cos_theta = cosf(_theta);
    const float sin_alpha = sinf(_alpha);
    const float cos_alpha = cosf(_alpha);

    Eigen::Matrix4f trans;
    trans << cos_theta, -sin_theta * cos_alpha, sin_theta* sin_alpha, _a* cos_theta,
        sin_theta, cos_theta* cos_alpha, -cos_theta * sin_alpha, _a* sin_theta,
        0.f, sin_alpha, cos_alpha, _d,
        0.f, 0.f, 0.f, 1.f;

    return trans;
}

const Eigen::Matrix4f CLinkDH::transform() const
{
    return transform(theta, alpha, a, d);
}

const CPose CLinkDH::pose() const
{
    return CPose(transform());
}

CDhArmModel::CDhArmModel()
{}

CDhArmModel::CDhArmModel(const std::string& _name) :
    m_sName(_name)
{}

std::vector<CLinkDH>* CDhArmModel::getArmDH()
{ 
    return &m_DHArmModel; 
}

const std::vector<CLinkDH>* CDhArmModel::getArmDH() const
{
    return &m_DHArmModel;
}

const std::vector<CPose> CDhArmModel::getKinematicChain() const
{
    std::vector<CPose> kinematic_chain;
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

    for (CyC_INT i = 0; i < m_DHArmModel.size(); ++i)
    {
        trans = trans * m_DHArmModel[i].transform();
        kinematic_chain.emplace_back(CPose(trans));
    }

    return kinematic_chain;
}

const void CDhArmModel::state2kinematicchain(const Eigen::VectorXf& state, std::vector<CPose>& kinematic_chain)
{
    kinematic_chain.clear();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

    for (CyC_INT i = 0; i < (state.size() / NO_DH_PARAMS); ++i)
    {
        float theta = state[i * NO_DH_PARAMS];
        float alpha = state[i * NO_DH_PARAMS + 1];
        float a = state[i * NO_DH_PARAMS + 2];
        float d = state[i * NO_DH_PARAMS + 3];

        trans = trans * CLinkDH::transform(theta, alpha, a, d);
        kinematic_chain.push_back(CPose(trans));
    }
}

CDhRobotModel::CDhRobotModel()
{}

CDhRobotModel::CDhRobotModel(const std::string& _robot_model_file)
{
    if (loadKinematics(_robot_model_file))
        m_bIsInitialised = true;
}

bool CDhRobotModel::loadKinematics(const std::string& _robot_model_file)
{
    this->clear();

    if (fs::exists(_robot_model_file.c_str()))
    {
        libconfig::Config configFile;

        configFile.readFile(_robot_model_file.c_str());

        const libconfig::Setting& rootConfig = configFile.getRoot();

        // Read body pose
        if (!rootConfig.exists("BodyPose") || rootConfig["BodyPose"].getLength() != 6)
        {
            spdlog::error("CDhRobotModel: The BodyPose must be defined using 6 DoF.");
            return false;
        }

        const libconfig::Setting& bodyPose = rootConfig["BodyPose"];
        const float x = bodyPose[0];
        const float y = bodyPose[1];
        const float z = bodyPose[2];
        const float roll = bodyPose[3];
        const float pitch = bodyPose[4];
        const float yaw = bodyPose[5];
        m_BodyPose.update(x, y, z, roll, pitch, yaw);
        
        // Read arm poses and DH configurations
        const libconfig::Setting& armsConfig = rootConfig["Arms"];
        for (CyC_INT i = 0; i < armsConfig.getLength(); ++i)
        {
            CDhArmModel dh_arm_model(armsConfig[i].getName());

            const libconfig::Setting& pose = armsConfig[i]["Pose"];
            if (pose.getLength() != 6)
            {
                spdlog::error("CDhRobotModel: The Pose must be defined using 6 DoF.");
                return false;
            }

            const float x = pose[0];
            const float y = pose[1];
            const float z = pose[2];
            const float roll = pose[3];
            const float pitch = pose[4];
            const float yaw = pose[5];
            dh_arm_model.m_Arm2BasePose.update(x, y, z, roll, pitch, yaw);

            const libconfig::Setting& Links = armsConfig[i]["Links"];
            for (CyC_INT j = 0; j < Links.getLength(); ++j)
            {
                float theta, alpha, a, d;
                const libconfig::Setting& LinksConfig = Links[j];
                LinksConfig.lookupValue("theta", theta);
                LinksConfig.lookupValue("alpha", alpha);
                LinksConfig.lookupValue("a", a);
                LinksConfig.lookupValue("d", d);

                dh_arm_model.m_DHArmModel.emplace_back(theta, alpha, a, d);
            }

            m_Arms.push_back(dh_arm_model);
        }
    }
    else
    {
        spdlog::error("CDhRobotModel: Robot model file does not exist.");
        return false;
    }

    m_bIsInitialised = true;
    return true;
}

void CDhRobotModel::clear()
{
    for (auto& arm : m_Arms)
        arm.m_DHArmModel.clear();

    m_Arms.clear();
}

CyC_INT CDhRobotModel::getNumLinks()
{
    CyC_INT num_links = 0;

    for (const auto& arm : m_Arms)
        num_links += (CyC_INT)arm.m_DHArmModel.size();

    return num_links;
}

std::vector<CPose> CDhRobotModel::dkine_feet()
{
    std::vector<CPose> end_effectors;
    for (size_t i = 0; i < m_Arms.size(); ++i)
    {
        CycPoses chain = m_Arms[i].getKinematicChain();
        end_effectors.emplace_back(m_Arms[i].m_Arm2BasePose * chain[chain.size() - 1]);
    }
    

    return end_effectors;
}

CPose CDhRobotModel::dkine_body(const Eigen::Vector3f& _foot_FL, const Eigen::Vector3f& _foot_FR, const Eigen::Vector3f& _foot_RL, const Eigen::Vector3f& _foot_RR)
{  
    std::vector<CPose> feet = dkine_feet();

    Eigen::Matrix4f DH = CPose::Rt2T(0, PI / 2, 0.f, 0.f, 0.f, 0.f);

    feet[0].update(DH * feet[0].transform());
    feet[1].update(DH * feet[1].transform());
    feet[2].update(DH * feet[2].transform());
    feet[3].update(DH * feet[3].transform());
    
    Eigen::MatrixXf plane_prev(3,4);
    plane_prev << _foot_FL, _foot_FR, _foot_RL, _foot_RR;

    Eigen::MatrixXf plane_pos(3, 4);
    plane_pos << feet[0].translation_3x1(), feet[1].translation_3x1(), feet[2].translation_3x1(), feet[3].translation_3x1();

    Eigen::Vector3f centroid_plane_prev(3, 1);
    centroid_plane_prev = (_foot_FL + _foot_FR + _foot_RL + _foot_RR) / 4;

    Eigen::Vector3f centroid_plane_pos(3, 1);
    centroid_plane_pos = (feet[0].translation_3x1() + feet[1].translation_3x1() + feet[2].translation_3x1() + feet[3].translation_3x1()) / 4;

    Eigen::Matrix3f H;
    H = ((plane_pos).colwise() - centroid_plane_pos) * ((plane_prev).colwise() - centroid_plane_prev).transpose();

    Eigen::JacobiSVD<Eigen::Matrix3f> svd;
    svd.compute(H, Eigen::ComputeThinV | Eigen::ComputeThinU);
    Eigen::Matrix3f U = svd.matrixU(), V = svd.matrixV();
    
    Eigen::Matrix3f R = V * U.transpose();

    if (R.determinant() < 0)
    {
        V.col(2) = V.col(2) * -1;
        R = V * U.transpose();
    }

    Eigen::Vector3f T = centroid_plane_prev - R * centroid_plane_pos;

    // din cauza schimabrilor de axe sunt ceva schimbari de coodronate
    Eigen::Matrix3f Rot = R.transpose();
    Eigen::Vector3f rq = CPose::R2euler(Rot);
    CPose O = CPose(-T(2), T(1), T(0), rq(2), -rq(1), -rq(0));

    O = CPose(DH * O.transform());

    //std::cout << O << std::endl << std::endl;
    //std::cout << O.transform() << std::endl << std::endl;


    return O;
}
