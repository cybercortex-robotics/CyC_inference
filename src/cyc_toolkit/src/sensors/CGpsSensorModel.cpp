// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CGpsSensorModel.h"

CGpsSensorModel::CGpsSensorModel(const std::string& calibration_file) :
    CBaseSensorModel(calibration_file)
{
    // Check if the calibration file exists
    if (!CFileUtils::FileExist(calibration_file.c_str()))
    {
        spdlog::error("{}: Calibration file does not exist. Sensor model could not be loaded.", typeid(*this).name());
    }
    else
    {
        if (this->loadSensorModel(calibration_file))
            spdlog::info("{}: Calibration loaded from \"{}\"", typeid(*this).name(), calibration_file);
        else
            spdlog::warn("{}: Could not load sensor calibration from \"{}\". CGpsSensorModel sensor disabled.", typeid(*this).name(), calibration_file);
    }
}

CGpsSensorModel::~CGpsSensorModel()
{}

bool CGpsSensorModel::loadSensorModel(const std::string& calibration_file)
{
    if (fs::exists(calibration_file.c_str()))
    {
        libconfig::Config configFile;
        configFile.readFile(calibration_file.c_str());

        const libconfig::Setting& rootConfig = configFile.getRoot();

        // Asked for rather than assumed: a file without it would come back as a libconfig
        // exception, and the caller would be left holding the zeros below without ever
        // being told they are not a very good receiver.
        if (!rootConfig.exists("Noise"))
        {
            spdlog::error("{}: Calibration '{}' has no Noise block.", typeid(*this).name(), calibration_file);
            return false;
        }

        const libconfig::Setting& Noise = rootConfig["Noise"];
        Noise.lookupValue("hpos_noise_stddev", m_StddevHorizontal);
        Noise.lookupValue("vpos_noise_stddev", m_StddevVertical);
        Noise.lookupValue("hpos_bias_stddev", m_BiasHorizontal);
        Noise.lookupValue("vpos_bias_stddev", m_BiasVertical);
        Noise.lookupValue("bias_correlation_time", m_BiasTau);

        const float nh2 = m_StddevHorizontal * m_StddevHorizontal;
        const float nv2 = m_StddevVertical * m_StddevVertical;

        // East and north carry the same variance: a receiver solves both from the same
        // spread of satellites, and nothing about its error prefers a direction.
        m_CovNoise.diagonal() << nh2, nh2, nv2;

        m_bInitialized = true;

        return true;
    }
    else
    {
        return false;
    }
}
