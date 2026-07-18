// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CRobotConfig.h"
#include <algorithm>
#include <iostream>
#include <spdlog/spdlog.h>
#include <os/CFileUtils.h>

CRobotConfig::CRobotConfig(const std::string& _robot_config_file)
{
    m_bIsInitialized = false;

    // Check if file exists
    bool bFileExists = CFileUtils::FileExist(_robot_config_file.c_str());

    if (bFileExists)
    {
        fs::path robot_config_path(_robot_config_file);
        if (robot_config_path.extension().compare(".conf") == 0)
            m_bIsInitialized = loadFromConfig(_robot_config_file);
        else
            spdlog::error("{}: Unknown robot config file type '{}'", typeid(*this).name(), robot_config_path.extension().string());
    }
    else
    {
        spdlog::error("{}: Robot config file does not exist: '{}'", typeid(*this).name(), _robot_config_file);
    }
}

CRobotConfig::~CRobotConfig()
{}

int CRobotConfig::baseStateSlot(const std::string& _field) const
{
    const auto it = std::find(base_state.begin(), base_state.end(), _field);
    if (it == base_state.end())
        return -1;

    return static_cast<int>(std::distance(base_state.begin(), it));
}

float CRobotConfig::baseStateValue(
    const CycState& _state, const std::string& _field, float _fallback) const
{
    const int slot = baseStateSlot(_field);

    return (slot < 0) ? _fallback : _state.x_hat[slot];
}

int CRobotConfig::jointStateSlot(const std::string& _field, int _dof) const
{
    const auto it = std::find(joint_state.begin(), joint_state.end(), _field);
    if (it == joint_state.end())
        return -1;

    const int field = static_cast<int>(std::distance(joint_state.begin(), it));

    return static_cast<int>(base_state.size()) + field * static_cast<int>(actuators.size()) + _dof;
}

bool CRobotConfig::loadFromConfig(const std::string& _robot_config_file)
{
    libconfig::Config LibConfigFile;

    try
    {
        LibConfigFile.readFile(_robot_config_file.c_str());
    }
    catch (const libconfig::ParseException& ex)
    {
        spdlog::error("{}: Failed to parse '{}': {} at line {}", typeid(*this).name(),
            _robot_config_file, ex.getError(), ex.getLine());
        return false;
    }
    catch (const libconfig::FileIOException&)
    {
        spdlog::error("{}: I/O error reading '{}'", typeid(*this).name(), _robot_config_file);
        return false;
    }

    const libconfig::Setting& root = LibConfigFile.getRoot();

    // --- Scalar header fields ------------------------------------------------
    root.lookupValue("robot", robot);
    root.lookupValue("description", description);
    root.lookupValue("base", base);

    // --- Actuators (ordered DoFs) -------------------------------------------
    actuators.clear();
    name2idx.clear();
    if (root.exists("Actuators"))
    {
        const libconfig::Setting& acts = root["Actuators"];
        for (int i = 0; i < acts.getLength(); ++i)
        {
            const libconfig::Setting& a = acts[i];

            Actuator dof;
            if (!a.lookupValue("name", dof.name) || dof.name.empty())
            {
                spdlog::warn("{}: Actuator[{}] has no 'name', skipped.", typeid(*this).name(), i);
                continue;
            }

            a.lookupValue("kp",      dof.kp);
            a.lookupValue("kd",      dof.kd);
            a.lookupValue("q_min",   dof.q_min);
            a.lookupValue("q_max",   dof.q_max);
            a.lookupValue("tau_min", dof.tau_min);
            a.lookupValue("tau_max", dof.tau_max);

            name2idx[dof.name] = static_cast<CyC_INT>(actuators.size());
            actuators.push_back(std::move(dof));
        }
    }

    // --- State layout (X) ----------------------------------------------------
    // 'State' lists two blocks: "base" contributes one x_hat slot per field, and
    // "joints" one slot per field per actuator. A field that is not listed is
    // neither published nor addressable.
    base_state.clear();
    joint_state.clear();
    if (root.exists("State"))
    {
        const libconfig::Setting& blocks = root["State"];
        for (int i = 0; i < blocks.getLength(); ++i)
        {
            const libconfig::Setting& block = blocks[i];

            std::string strName;
            if (!block.lookupValue("name", strName))
            {
                spdlog::warn("{}: State[{}] has no 'name', skipped.", typeid(*this).name(), i);
                continue;
            }

            std::vector<std::string>* fields = nullptr;
            if (strName == "base")
                fields = &base_state;
            else if (strName == "joints")
                fields = &joint_state;
            else
            {
                spdlog::warn("{}: State[{}]: unknown block '{}', expected 'base' or 'joints'.",
                    typeid(*this).name(), i, strName);
                continue;
            }

            if (!block.exists("fields"))
            {
                spdlog::warn("{}: State block '{}' has no 'fields', skipped.",
                    typeid(*this).name(), strName);
                continue;
            }

            const libconfig::Setting& listed = block["fields"];
            for (int f = 0; f < listed.getLength(); ++f)
                fields->push_back(static_cast<const char*>(listed[f]));
        }
    }

    // --- Control input layout (u) --------------------------------------------
    // 'Input' lists the terms of the control vector sent to each actuator, in
    // slot order, so slot f of u is control_input[f]. A term that is not listed
    // is not commandable; an adapter reads it as 0.
    control_input.clear();
    if (root.exists("Input"))
    {
        const libconfig::Setting& listed = root["Input"];
        for (int f = 0; f < listed.getLength(); ++f)
            control_input.push_back(static_cast<const char*>(listed[f]));
    }

    // --- Control dimensions (X, U, Y) ---------------------------------------
    // Read explicitly if present (e.g. a fixed vehicle/drone model), otherwise
    // default the input dimension to the number of actuated DoFs (direct drive)
    // and the state dimension to the layout declared in 'State'.
    root.lookupValue("num_states", num_states);
    root.lookupValue("num_inputs", num_inputs);
    root.lookupValue("num_outputs", num_outputs);

    if (num_inputs == 0)
        num_inputs = static_cast<int>(actuators.size());

    if (!base_state.empty() || !joint_state.empty())
        num_states = static_cast<int>(base_state.size() + joint_state.size() * actuators.size());

    spdlog::info("{}: loaded '{}' (actuators={}, u_dim={}, X/U/Y={}/{}/{})", typeid(*this).name(),
        robot.empty() ? "<unnamed>" : robot, actuators.size(), control_input.size(),
        num_states, num_inputs, num_outputs);

    return true;
}
