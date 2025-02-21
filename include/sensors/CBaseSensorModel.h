// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CBaseSensorModel_H_
#define CBaseSensorModel_H_

#include "CCR_TYPES.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"

class CBaseSensorModel
{
public:
    explicit CBaseSensorModel(const std::string& calibration_file);
    virtual ~CBaseSensorModel();

    const std::string& getCalibrationFile() const;
    const CPose& pose() const;
    const CPose& extrinsics() const;
    const CPose& extrinsics_inv() const;
    void updatePose(const CPose& _pose);

    /// Project from sensor to world coordiantes. Returns a bearing vector of unit length.
    virtual Eigen::Vector3f sensor2world(const float& x, const float& y) const = 0;

    /// Project from sensor to world coordiantes. Returns a bearing vector of unit length.
    virtual Eigen::Vector3f sensor2world(const Eigen::Vector2f& px) const = 0;

    virtual Eigen::Vector2f world2sensor(const Eigen::Vector3f& xyz_c) const = 0;
    virtual Eigen::Vector2f world2sensor(const Eigen::Vector4f& xyz_c) const = 0;

    /// projects unit plane coordinates to sensor coordinates
    virtual Eigen::Vector2f world2sensor(const Eigen::Vector2f& uv) const = 0;

    virtual float errorMultiplier2() const = 0;
    virtual float errorMultiplier() const = 0;

private:
    virtual bool loadSensorModel(const std::string& calibration_file) = 0;

private:
    std::string m_sCalibrationFile;
    CPose       m_Pose;       // updated continuous by via state estimation in the respective state estimation filter
    CPose       m_Extrinsics; // fixed parameters read from the calibration files, describing the mounting place of the sensor wrt the robot's coordintate frame
    CPose       m_Extrinsics_inv;
};

#endif /* CBaseSensorModel_H_ */
