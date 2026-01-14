// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CBaseSensorModel.h"
#include "vision/CProjectiveGeometry.h"

CBaseSensorModel::CBaseSensorModel(const std::string& calibration_file)
{
    // Check if the calibration file exists
    if (!CFileUtils::FileExist(calibration_file.c_str()))
    {
        spdlog::error("{}: Calibration file '{}' does not exist.", typeid(*this).name(), calibration_file);
    }
    else
    {
        m_sCalibrationFile = calibration_file;

        float rot_x, rot_y, rot_z, trans_x, trans_y, trans_z;
        libconfig::Config configFile;
        configFile.readFile(calibration_file.c_str());
        
        const libconfig::Setting& RootConfig = configFile.getRoot();

        // Check if a given pose exists
        if (RootConfig.exists("Pose"))
        {
            const libconfig::Setting& PoseConfig = RootConfig["Pose"];
            const libconfig::Setting& RotationConfig = PoseConfig["Rotation"];
            RotationConfig.lookupValue("x", rot_x);
            RotationConfig.lookupValue("y", rot_y);
            RotationConfig.lookupValue("z", rot_z);
            const libconfig::Setting& TranslationConfig = PoseConfig["Translation"];
            TranslationConfig.lookupValue("x", trans_x);
            TranslationConfig.lookupValue("y", trans_y);
            TranslationConfig.lookupValue("z", trans_z);

            // Calculate transformation matrices
            rot_x *= DEG2RAD;
            rot_y *= DEG2RAD;
            rot_z *= DEG2RAD;
            m_Extrinsics.update(trans_x, trans_y, trans_z, rot_x, rot_y, rot_z);
            m_Extrinsics_inv = CPose(CProjectiveGeometry::invertT(m_Extrinsics.transform()));
            m_Pose.update(trans_x, trans_y, trans_z, rot_x, rot_y, rot_z);
        }
    }
}

CBaseSensorModel::~CBaseSensorModel() 
{}

const std::string& CBaseSensorModel::getCalibrationFile() const 
{
    return m_sCalibrationFile;
}

const CPose& CBaseSensorModel::pose() const
{
    return m_Pose;
}

const CPose& CBaseSensorModel::extrinsics() const
{
    return m_Extrinsics;
}

const CPose& CBaseSensorModel::extrinsics_inv() const
{
    return m_Extrinsics_inv;
}

void CBaseSensorModel::updatePose(const CPose& _pose)
{
    m_Pose = _pose;
}

void CBaseSensorModel::updateExtrinsics(const Eigen::Matrix4f& _pose)
{
    m_Extrinsics = CPose(_pose);
    m_Extrinsics_inv = m_Extrinsics.inverse();
}

void CBaseSensorModel::updateExtrinsics(const CPose& _pose)
{
    m_Extrinsics = _pose;
    m_Extrinsics_inv = _pose.inverse();
}
