// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CImuSensorModel_H_
#define CImuSensorModel_H_

#include "CyC_TYPES.h"
#include <Eigen/Eigen>
#include "CBaseSensorModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"

// IMU gyro and accelerometer biases
class Bias
{
public:
    Bias() = default;
    Bias(float _bias_acc_x, float _bias_acc_y, float _bias_acc_z,
        float _bias_gyro_x, float _bias_gyro_y, float _bias_gyro_z)
    {
        m_BiasAcc = Eigen::Vector3f(_bias_acc_x, _bias_acc_y, _bias_acc_z);
        m_BiasGyro = Eigen::Vector3f(_bias_gyro_x, _bias_gyro_y, _bias_gyro_z);
    }
    Bias(const Eigen::Vector3f& _bias_acc, const Eigen::Vector3f& _bias_gyro) :
        m_BiasAcc(_bias_acc), m_BiasGyro(_bias_gyro)
    {}

    void CopyFrom(const Bias& _b)
    {
        m_BiasAcc = _b.m_BiasAcc;
        m_BiasGyro = _b.m_BiasGyro;
    }

    friend auto operator<<(std::ostream& _os, Bias const& _bias) -> std::ostream&
    {
        return _os << "Acc: [" << _bias.m_BiasAcc.x() << "\t" << _bias.m_BiasAcc.y() << "\t" << _bias.m_BiasAcc.z() << "]\t Gyro: [" << _bias.m_BiasGyro.x() << "\t" << _bias.m_BiasGyro.y() << "\t" << _bias.m_BiasGyro.z() << "]";
    }

public:
    static Bias Bias_DEFAULT;
    Eigen::Vector3f m_BiasAcc = Eigen::Vector3f::Zero();
    Eigen::Vector3f m_BiasGyro = Eigen::Vector3f::Zero();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CImuSensorModel : public CBaseSensorModel 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CImuSensorModel(const std::string& calibration_file);
    ~CImuSensorModel();

    virtual Eigen::Vector3f sensor2world(const float& x, const float& y) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector3f depth2world(const float& x_d, const float& y_d, const float& depth) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector3f sensor2world(const Eigen::Vector2f& px) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector2f world2sensor(const Eigen::Vector3f& xyz) const { return Eigen::Vector2f{ 0.f, 0.f }; };
    virtual Eigen::Vector2f world2sensor(const Eigen::Vector4f& xyz) const { return Eigen::Vector2f{ 0.f, 0.f }; };

    virtual Eigen::Vector2f world2sensor(const Eigen::Vector2f& uv) const { return Eigen::Vector2f{ 0.f, 0.f }; };

    virtual float errorMultiplier2() const { return 0.f; };
    virtual float errorMultiplier() const { return 0.f; };

    float getNoiseAcc() const { return m_StddevAcc; }
    float getNoiseGyro() const { return m_StddevGyro; }
    float getRandwalkAcc() const { return m_RandwalkAcc; }
    float getRandwalkGyro() const { return m_RandwalkGyro; }
    Eigen::DiagonalMatrix<float, 6> getCovNoise() const { return m_CovNoise; };
    Eigen::DiagonalMatrix<float, 6> getCovRandWalk() const { return m_CovRandWalk; };

    bool isInitialized() const { return m_bInitialized; };
    void setInitialized(const bool& _initialized) { m_bInitialized = _initialized; };
    
private:
    virtual bool loadSensorModel(const std::string& calibration_file);

private:
    bool m_bInitialized = false;
    
    // Noise
    float m_StddevAcc = 0.f;
    float m_StddevGyro = 0.f;
    float m_RandwalkAcc = 0.f;
    float m_RandwalkGyro = 0.f;
    Eigen::DiagonalMatrix<float, 6> m_CovNoise;
    Eigen::DiagonalMatrix<float, 6> m_CovRandWalk;
};


#endif /* CImuSensorModel_H_ */
