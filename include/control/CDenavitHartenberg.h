/*
 * CDenavitHartenberg.h
 *
 *  Created on: 08.10.2022
 *      Author: Sorin Grigorescu
 */

#ifndef CDenavitHartenberg_H_
#define CDenavitHartenberg_H_

#include <CyC_TYPES.h>
#include "CStateSpaceModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"

#define NO_DH_PARAMS 4

class CLinkDH
{
public:
    CLinkDH();
    CLinkDH(float _theta, float _alpha, float _a, float _d);

    static const Eigen::Matrix4f transform(float _theta, float _alpha, float _a, float _d);

    const Eigen::Matrix4f transform() const;

    const CPose pose() const;

public:
    float theta;
    float alpha;
    float a;
    float d;

    // Relevant for dynamics only
    float theta_dot = 0.f;
    float theta_ddot = 0.f;
};

class CDhArmModel
{
public:
    CDhArmModel();
    CDhArmModel(const std::string& _name);

    std::vector<CLinkDH>* getArmDH();
    const std::vector<CLinkDH>* getArmDH() const;

    const std::vector<CPose> getKinematicChain() const;

    static const void state2kinematicchain(const Eigen::VectorXf& state, std::vector<CPose>& kinematic_chain);

public:
    CPose                   m_Arm2BasePose;
    std::string              m_sName = "";
    std::vector<CLinkDH>    m_DHArmModel;
};

class CDhRobotModel
{
public:
    CDhRobotModel();
    CDhRobotModel(const std::string& _robot_model_file);

    bool isInitialised() { return m_bIsInitialised; };

    CPose getBodyPose() const { return m_BodyPose; };
    void setBodyPose(const CPose& _pose) { m_BodyPose = _pose; };
    
    CyC_INT getNumLinks();
    
    bool loadKinematics(const std::string& _robot_model_file);

    std::vector<CPose> dkine_feet();
    CPose dkine_body(const Eigen::Vector3f& _foot_FL, const Eigen::Vector3f& _foot_FR, const Eigen::Vector3f& _foot_RL, const Eigen::Vector3f& _foot_RR);

    void clear();

public:
    bool m_bIsInitialised = false;

    // Body reference pose
    CPose m_BodyPose;

    // DH model of each arm
    std::vector<CDhArmModel> m_Arms;
};

#endif /* CDenavitHartenberg_H_ */
