// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CLidarSensorModel.h"

CLidarSensorModel::CLidarSensorModel(const std::string& calibration_file) :
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
            spdlog::warn("{}: Could not load sensor calibration from \"{}\". CLidarSensorModel sensor disabled.", typeid(*this).name(), calibration_file);
    }
}

CLidarSensorModel::~CLidarSensorModel()
{}

bool CLidarSensorModel::loadSensorModel(const std::string& calibration_file)
{
    if (fs::exists(calibration_file.c_str()))
    {
        libconfig::Config configFile;
        configFile.readFile(calibration_file.c_str());

        const libconfig::Setting& rootConfig = configFile.getRoot();

        if (rootConfig.exists("range"))
        {
            const libconfig::Setting& range = rootConfig.lookup("range");
            min_range_ = static_cast<float>(range[0]);
            max_range_ = static_cast<float>(range[1]);
        }
        else
        {
            spdlog::error("Lidar range parameter has to be set in '{}'. Exiting.", calibration_file);
            exit(EXIT_FAILURE);
        }

        if (rootConfig.exists("num_points"))
        {
            rootConfig.lookupValue("num_points", num_points_);
        }
        else
        {
            spdlog::error("Lidar num_points parameter has to be set in '{}'. Exiting.", calibration_file);
            exit(EXIT_FAILURE);
        }

        if (rootConfig.exists("config_file"))
            rootConfig.lookupValue("config_file", config_file_);
        
        return true;
    }
    else
    {
        return false;
    }
}
