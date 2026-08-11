// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CBarometerSensorModel.h"

CBarometerSensorModel::CBarometerSensorModel(const std::string& calibration_file) :
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
            spdlog::warn("{}: Could not load sensor calibration from \"{}\". CBarometerSensorModel sensor disabled.", typeid(*this).name(), calibration_file);
    }
}

CBarometerSensorModel::~CBarometerSensorModel()
{}

bool CBarometerSensorModel::loadSensorModel(const std::string& calibration_file)
{
    if (fs::exists(calibration_file.c_str()))
    {
        libconfig::Config configFile;
        configFile.readFile(calibration_file.c_str());

        const libconfig::Setting& rootConfig = configFile.getRoot();

        // Asked for rather than assumed: a file without it would come back as a libconfig
        // exception, and the caller would be left holding the zeros below without ever
        // being told they are not a very quiet barometer.
        if (!rootConfig.exists("Noise"))
        {
            spdlog::error("{}: Calibration '{}' has no Noise block.", typeid(*this).name(), calibration_file);
            return false;
        }

        const libconfig::Setting& Noise = rootConfig["Noise"];
        Noise.lookupValue("altitude_noise_stddev", m_StddevAltitude);
        Noise.lookupValue("vario_noise_stddev", m_StddevVario);
        Noise.lookupValue("altitude_noise_randwalk", m_RandwalkAltitude);

        const float na2 = m_StddevAltitude * m_StddevAltitude;
        const float nv2 = m_StddevVario * m_StddevVario;

        m_CovNoise.diagonal() << na2, nv2;

        m_bInitialized = true;

        return true;
    }
    else
    {
        return false;
    }
}
