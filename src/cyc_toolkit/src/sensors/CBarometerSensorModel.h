// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CBarometerSensorModel_H_
#define CBarometerSensorModel_H_

#include "CyC_TYPES.h"
#include <Eigen/Eigen>
#include "CBaseSensorModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"

// The calibration of a barometric altimeter: where it is mounted, and how much it lies.
//
// A barometer has no geometry to calibrate the way a camera or a lidar does -- it projects
// nothing -- so the projection methods below answer with zeros, as CImuSensorModel's do.
// What it has is a noise, and that is what anything fusing it needs to know.
//
// The calibration file holds:
//
//   Pose = { Rotation = {...}; Translation = {...}; }   read by CBaseSensorModel
//   Noise =
//   {
//       altitude_noise_stddev   = 0.5;    // [m]
//       vario_noise_stddev      = 0.1;    // [m/s]
//       altitude_noise_randwalk = 0.05;   // [m / sqrt(s)]
//   }
//
// The random walk is what makes a barometer drift: the ambient pressure wanders over a
// flight and an altimeter has no way to tell the weather from the aircraft descending. It
// is most of why anything downstream fuses this sensor rather than believing it.
class CBarometerSensorModel : public CBaseSensorModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CBarometerSensorModel(const std::string& calibration_file);
    ~CBarometerSensorModel();

    virtual Eigen::Vector3f sensor2world(const float& x, const float& y) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector3f depth2world(const float& x_d, const float& y_d, const float& depth) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector3f sensor2world(const Eigen::Vector2f& px) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector2f world2sensor(const Eigen::Vector3f& xyz) const { return Eigen::Vector2f{ 0.f, 0.f }; };
    virtual Eigen::Vector2f world2sensor(const Eigen::Vector4f& xyz) const { return Eigen::Vector2f{ 0.f, 0.f }; };

    virtual Eigen::Vector2f world2sensor(const Eigen::Vector2f& uv) const { return Eigen::Vector2f{ 0.f, 0.f }; };

    virtual float errorMultiplier2() const { return 0.f; };
    virtual float errorMultiplier() const { return 0.f; };

    // Per-sample measurement noise, as standard deviations.
    float getNoiseAltitude() const { return m_StddevAltitude; }  // [m]
    float getNoiseVario() const { return m_StddevVario; }        // [m/s]

    // Bias instability, as the density of a random walk: over a step of dt the bias moves
    // by this times sqrt(dt).
    float getRandwalkAltitude() const { return m_RandwalkAltitude; }  // [m / sqrt(s)]

    // The measurement covariance of a reading, over {altitude, vertical speed} -- the two
    // numbers a barometer filter publishes, in the order it publishes them.
    Eigen::DiagonalMatrix<float, 2> getCovNoise() const { return m_CovNoise; };

    // False when the file held no Noise block, so the zeros below are not a quiet barometer
    // but a calibration that was never read. Worth asking before believing them.
    bool isInitialized() const { return m_bInitialized; };
    void setInitialized(const bool& _initialized) { m_bInitialized = _initialized; };

private:
    virtual bool loadSensorModel(const std::string& calibration_file);

private:
    bool m_bInitialized = false;

    // Noise
    float m_StddevAltitude = 0.f;
    float m_StddevVario = 0.f;
    float m_RandwalkAltitude = 0.f;
    Eigen::DiagonalMatrix<float, 2> m_CovNoise;
};

#endif /* CBarometerSensorModel_H_ */
