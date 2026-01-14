// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CImuSensorModel.h"

Bias Bias::Bias_DEFAULT = Bias();

CImuSensorModel::CImuSensorModel(const std::string& calibration_file) :
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

CImuSensorModel::~CImuSensorModel()
{}

bool CImuSensorModel::loadSensorModel(const std::string& calibration_file)
{
    if (fs::exists(calibration_file.c_str()))
    {
        libconfig::Config configFile;
        configFile.readFile(calibration_file.c_str());

        const libconfig::Setting& rootConfig = configFile.getRoot();

        // Check if an imu to camera transformation matrix is given
        if (rootConfig.exists("T_ci"))
        {
            bool success = true;
            Eigen::Matrix4f T_ci;
            const libconfig::Setting& T_ci_setting = rootConfig["T_ci"];

            if (T_ci_setting.isList() && T_ci_setting.getLength() == 4)
            {
                for (int i = 0; i < 4; ++i)
                {
                    const libconfig::Setting& row = T_ci_setting[i];
                    if (row.isList() && row.getLength() == 4)
                    {
                        for (int j = 0; j < 4; ++j)
                            T_ci(i, j) = row[j];
                    }
                    else
                    {
                        spdlog::error("{}: Reading T_ci: Row {} is not a valid array or has incorrect length.", typeid(*this).name(), i);
                        success = false;
                    }
                }
            }
            else
            {
                spdlog::error("{}: Reading T_ci: 'T_ci' is not a 4x4 matrix.", typeid(*this).name());
                success = false;
            }

            if (success)
            {
                this->updateExtrinsics(T_ci);
                this->updatePose(T_ci);
            }
        }

        const libconfig::Setting& Noise = rootConfig["Noise"];
        Noise.lookupValue("acc_noise_stddev", m_StddevAcc);
        Noise.lookupValue("gyro_noise_stddev", m_StddevGyro);
        Noise.lookupValue("acc_noise_randwalk", m_RandwalkAcc);
        Noise.lookupValue("gyro_noise_randwalk", m_RandwalkGyro);

        const float na2 = m_StddevAcc * m_StddevAcc;
        const float ng2 = m_StddevGyro * m_StddevGyro;
        const float naw2 = m_RandwalkAcc * m_RandwalkAcc;
        const float ngw2 = m_RandwalkGyro * m_RandwalkGyro;

        m_CovNoise.diagonal() << ng2, ng2, ng2, na2, na2, na2;
        m_CovRandWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;

        return true;
    }
    else
    {
        return false;
    }
}
