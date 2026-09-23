// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CGpsSensorModel_H_
#define CGpsSensorModel_H_

#include "CyC_TYPES.h"
#include <Eigen/Eigen>
#include "CBaseSensorModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"

// The calibration of a GNSS receiver: where its antenna is mounted, and how wrong its
// fixes are.
//
// A receiver projects nothing, so the projection methods below answer with zeros, as
// CImuSensorModel's and CBarometerSensorModel's do. What it has is an error, and that is
// what anything fusing it needs to know.
//
// The calibration file holds:
//
//   Pose = { Rotation = {...}; Translation = {...}; }   read by CBaseSensorModel
//   Noise =
//   {
//       hpos_noise_stddev     = 1.2;    // [m]
//       vpos_noise_stddev     = 2.4;    // [m]
//       hpos_bias_stddev      = 1.0;    // [m]
//       vpos_bias_stddev      = 2.0;    // [m]
//       bias_correlation_time = 100.0;  // [s]
//   }
//
// The noise is quoted in metres rather than in degrees on purpose: a degree of longitude
// is a different distance at every latitude, so a covariance in degrees could not be read
// without also knowing where on Earth it was taken.
//
// Vertical is the larger of the two because of the geometry: every satellite a receiver
// can see is above it, so the vertical component is solved from a narrower spread of
// directions than the horizontal one. A factor of about two is usual.
//
// The bias is the part of the error that does not average away. A receiver's error is
// mostly ionospheric delay and multipath, which wander over minutes rather than being
// redrawn at every fix -- standing still and averaging for a minute gives a tighter
// answer that is still in the wrong place. Unlike a barometer's, it is bounded: GNSS
// error wanders within a few metres and stays there, which is why this is a correlation
// time and a steady-state sigma rather than a random walk.
class CGpsSensorModel : public CBaseSensorModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CGpsSensorModel(const std::string& calibration_file);
    ~CGpsSensorModel();

    virtual Eigen::Vector3f sensor2world(const float& x, const float& y) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector3f depth2world(const float& x_d, const float& y_d, const float& depth) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector3f sensor2world(const Eigen::Vector2f& px) const { return Eigen::Vector3f{ 0.f, 0.f, 0.f }; };

    virtual Eigen::Vector2f world2sensor(const Eigen::Vector3f& xyz) const { return Eigen::Vector2f{ 0.f, 0.f }; };
    virtual Eigen::Vector2f world2sensor(const Eigen::Vector4f& xyz) const { return Eigen::Vector2f{ 0.f, 0.f }; };

    virtual Eigen::Vector2f world2sensor(const Eigen::Vector2f& uv) const { return Eigen::Vector2f{ 0.f, 0.f }; };

    virtual float errorMultiplier2() const { return 0.f; };
    virtual float errorMultiplier() const { return 0.f; };

    // Per-sample measurement noise, as standard deviations [m].
    float getNoiseHorizontal() const { return m_StddevHorizontal; }
    float getNoiseVertical() const { return m_StddevVertical; }

    // Where the slowly wandering part of the error settles [m], and how long it takes to
    // forget where it was [s].
    float getBiasHorizontal() const { return m_BiasHorizontal; }
    float getBiasVertical() const { return m_BiasVertical; }
    float getBiasCorrelationTime() const { return m_BiasTau; }

    // The measurement covariance of a fix, over {east, north, up} in metres. Horizontal
    // is one number for both east and north: a receiver's error has no compass sense.
    Eigen::DiagonalMatrix<float, 3> getCovNoise() const { return m_CovNoise; };

    // False when the file held no Noise block, so the zeros below are not a perfect
    // receiver but a calibration that was never read. Worth asking before believing them.
    bool isInitialized() const { return m_bInitialized; };
    void setInitialized(const bool& _initialized) { m_bInitialized = _initialized; };

private:
    virtual bool loadSensorModel(const std::string& calibration_file);

private:
    bool m_bInitialized = false;

    // Noise
    float m_StddevHorizontal = 0.f;
    float m_StddevVertical = 0.f;
    float m_BiasHorizontal = 0.f;
    float m_BiasVertical = 0.f;
    float m_BiasTau = 0.f;
    Eigen::DiagonalMatrix<float, 3> m_CovNoise;
};

#endif /* CGpsSensorModel_H_ */
