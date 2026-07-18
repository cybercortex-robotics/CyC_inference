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
//   - declares the control vector layout (control_input) and carries the
//     per-actuator metadata (name, gains, limits) that adapters use to translate
//     an abstract control input into a concrete backend command
//     (MuJoCo CycControlInputsQ, unitree LowCmd_, etc.),
//   - parameterises the state-space models (CBaseStateSpaceModel & derived).
// -----------------------------------------------------------------------------
class CRobotConfig
{
public:
    // One actuated degree of freedom. Order in Layout::actuators is authoritative:
    // index i here maps to index i in the joint portion of the Cyc*Q vectors.
    struct Actuator
    {
        std::string     name;                       // backend joint / actuator name
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
    int     numControlInput_Joint() const { return static_cast<int>(control_input.size()); };

    // Slot of a state field in x_hat, per the layout of base_state / joint_state
    // below; -1 if the descriptor does not list the field, which is also how a
    // caller learns that the field is neither published nor addressable.
    int     baseStateSlot(const std::string& _field) const;
    int     jointStateSlot(const std::string& _field, int _dof) const;

    // Value of a base state field read out of '_state.x_hat', or '_fallback' if
    // the descriptor does not list the field. Bridges the backend-agnostic slot
    // layout (baseStateSlot) to a concrete published state vector.
    float   baseStateValue(
        const CycState& _state, const std::string& _field, float _fallback = 0.f) const;

private:
    bool    loadFromConfig(const std::string& _robot_config_file);

public:
    std::string             robot;              // robot name / id
    std::string             description;        // description of the robot
    std::string             base;               // base body name
    std::vector<Actuator>   actuators;          // ordered DoFs

    // x_hat layout, from the descriptor's State block. The base fields come
    // first, one slot each; then one contiguous block per joint field, each
    // holding one slot per actuator in 'actuators' order:
    //
    //   x_hat = [ base_state | joint_state[0] over all DoFs | joint_state[1] ... ]
    //
    // so slot (base_state.size() + f * actuators.size() + i) is joint field f of
    // actuator i, and a slot index within a field block doubles as the DoF index.
    std::vector<std::string> base_state;    // per-base fields    (x, qw, vx, ...)
    std::vector<std::string> joint_state;   // per-actuator fields (q, qd, tau)

    // u layout, from the descriptor's Input block: the terms of the control
    // vector sent to each actuator, in slot order, so slot f of u is
    // control_input[f]. This is what interprets a command for a DoF -- e.g. a
    // position-PD joint declares ("kp", "kd", "q_des", "qd_des", "tau_ff"), a
    // rotor ("thrust"). A term that is not listed is not commandable.
    std::vector<std::string> control_input;

    // Control dimensions used to size the Cyc* vectors and state-space models.
    // num_states is derived from the State block; num_inputs defaults to
    // actuators.size() (direct-drive) unless the descriptor overrides it
    // (multirotor / wheeled control-allocation).
    int                     num_states  = 0;    // X  -> x_hat
    int                     num_inputs  = 0;    // U  -> u
    int                     num_outputs = 0;    // Y  -> y_hat

    // name -> index into 'actuators' (missing => not an actuated DoF).
    std::map<std::string, int> name2idx;

private:
    bool    m_bIsInitialized;
};

#endif /* CRobotConfig_H_ */
