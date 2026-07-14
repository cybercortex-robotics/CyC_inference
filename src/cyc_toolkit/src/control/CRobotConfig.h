// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CRobotConfig_H_
#define CRobotConfig_H_

#include <map>
#include <string>
#include <vector>
#include "CyC_TYPES.h"
#include "os/CConversions.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)

// -----------------------------------------------------------------------------
// CRobotConfig
//
// Parses a robot descriptor (*.conf, libconfig) into a backend-agnostic Layout.
// The Layout is the single source of truth that:
//   - sizes / labels the Cyc*Q vectors (num_states / num_inputs / num_outputs),
//   - carries per-actuator metadata (name, mode, gains, limits) that adapters
//     use to translate an abstract control input into a concrete backend command
//     (MuJoCo CycControlInputsQ, unitree LowCmd_, etc.),
//   - parameterises the state-space models (CBaseStateSpaceModel & derived).
// -----------------------------------------------------------------------------
class CRobotConfig
{
public:
    // Low-level command interpretation for a single actuator (DoF).
    enum ActuatorMode
    {
        Mode_UNDEFINED = 0,
        Mode_POSITION_PD,   // {kp, kd, q_des}   (arms, legs)
        Mode_VELOCITY,      // {kd, dq_des}      (drive wheels, rotors)
        Mode_TORQUE,        // {tau}             (direct torque)
        Mode_THRUST         // {thrust}          (rotor thrust)
    };

    // One actuated degree of freedom. Order in Layout::actuators is authoritative:
    // index i here maps to index i in the joint portion of the Cyc*Q vectors.
    struct Actuator
    {
        std::string     name;                       // backend joint / actuator name
        ActuatorMode    mode    = Mode_POSITION_PD;
        float           kp      = 0.f;              // default position gain
        float           kd      = 0.f;              // default damping gain
        float           q_min   = 0.f;              // position limits [rad | m]
        float           q_max   = 0.f;
        float           tau_min = 0.f;              // effort limits   [Nm | N]
        float           tau_max = 0.f;
    };

public:
    CRobotConfig(const std::string& _robot_config_file);
    virtual ~CRobotConfig();

    bool    isInitialized() const { return m_bIsInitialized; };
    static ActuatorMode String2ActuatorMode(const std::string& _mode);

private:
    bool loadFromConfig(const std::string& _robot_config_file);

public:
    std::string             robot;              // robot name / id
    std::string             description;        // description of the robot
    std::string             base;               // base body name
    std::vector<Actuator>   actuators;          // ordered DoFs

    // Control dimensions used to size the Cyc* vectors and state-space models.
    // num_inputs defaults to actuators.size() (direct-drive) unless the
    // descriptor overrides it (multirotor / wheeled control-allocation).
    CyC_INT                 num_states  = 0;    // X  -> x_hat
    CyC_INT                 num_inputs  = 0;    // U  -> u
    CyC_INT                 num_outputs = 0;    // Y  -> y_hat

    // name -> index into 'actuators' (missing => not an actuated DoF).
    std::map<std::string, CyC_INT> name2idx;

private:
    bool    m_bIsInitialized;
};

#endif /* CRobotConfig_H_ */
