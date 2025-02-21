// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CLidarSensorModel_H_
#define CLidarSensorModel_H_

#include "CCR_TYPES.h"
#include <Eigen/Eigen>
#include "CBaseSensorModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"


class CLidarSensorModel : public CBaseSensorModel 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CLidarSensorModel(const std::string& calibration_file);
    ~CLidarSensorModel();

    virtual Eigen::Vector3f sensor2world(const float& x, const float& y) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector3f depth2world(const float& x_d, const float& y_d, const float& depth) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector3f sensor2world(const Eigen::Vector2f& px) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector2f world2sensor(const Eigen::Vector3f& xyz) const { return Eigen::Vector2f{ 0.f, 0.f }; };
    virtual Eigen::Vector2f world2sensor(const Eigen::Vector4f& xyz) const { return Eigen::Vector2f{ 0.f, 0.f }; };

    virtual Eigen::Vector2f world2sensor(const Eigen::Vector2f& uv) const { return Eigen::Vector2f{ 0.f, 0.f }; };

    virtual float errorMultiplier2() const { return 0.f; };
    virtual float errorMultiplier() const { return 0.f; };
    
private:
    virtual bool loadSensorModel(const std::string& calibration_file);
};


#endif /* CLidarSensorModel_H_ */
