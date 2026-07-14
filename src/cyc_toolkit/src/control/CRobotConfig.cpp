// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CRobotConfig.h"
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

CRobotConfig::ActuatorMode CRobotConfig::String2ActuatorMode(const std::string& _mode)
{
    if (_mode == "position_pd" || _mode == "position")  return Mode_POSITION_PD;
    if (_mode == "velocity")                            return Mode_VELOCITY;
    if (_mode == "torque")                              return Mode_TORQUE;
    if (_mode == "thrust")                              return Mode_THRUST;
    return Mode_UNDEFINED;
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

            std::string strMode;
            if (a.lookupValue("mode", strMode))
                dof.mode = String2ActuatorMode(strMode);

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

    // --- Control dimensions (X, U, Y) ---------------------------------------
    // Read explicitly if present (e.g. a fixed vehicle/drone model), otherwise
    // default the input dimension to the number of actuated DoFs (direct drive).
    root.lookupValue("num_states", num_states);
    root.lookupValue("num_inputs", num_inputs);
    root.lookupValue("num_outputs", num_outputs);

    if (num_inputs == 0)
        num_inputs = static_cast<CyC_INT>(actuators.size());

    spdlog::info("{}: loaded '{}' (actuators={}, X/U/Y={}/{}/{})", typeid(*this).name(),
        robot.empty() ? "<unnamed>" : robot, actuators.size(), num_states, num_inputs, num_outputs);

    return true;
}
