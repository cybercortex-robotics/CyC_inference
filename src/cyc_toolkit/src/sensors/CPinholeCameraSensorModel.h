// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CPinholeCameraSensorModel_H_
#define CPinholeCameraSensorModel_H_

#include "CyC_TYPES.h"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "CBaseSensorModel.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include "os/CFileUtils.h"


class CPinholeCameraSensorModel : public CBaseSensorModel 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CPinholeCameraSensorModel(const std::string& _calibration_file, const bool& _is_right_camera = false);
    ~CPinholeCameraSensorModel();

    void init();
    void initUnistortionMap();
    
    virtual Eigen::Vector3f sensor2world(const float& x, const float& y) const;
    virtual Eigen::Vector3f sensor2world(const Eigen::Vector2f& px) const;
    virtual Eigen::Vector2f world2sensor(const Eigen::Vector2f& uv) const;
    virtual Eigen::Vector2f world2sensor(const Eigen::Vector3f& xyz) const;
    virtual Eigen::Vector2f world2sensor(const Eigen::Vector4f& xyz) const;
    virtual Eigen::Vector3f depth2world(const float& x_d, const float& y_d, const float& depth) const;

    const Eigen::Vector2f           focal_length() const        { return Eigen::Vector2f(fx_px_, fy_px_); }
    virtual float                   errorMultiplier2() const    { return fabs(fx_px_); }
    virtual float                   errorMultiplier() const     { return fabs(4.F*fx_px_*fy_px_); }
    inline const bool               isRightCam() const          { return bIsRightCamera_; }
    inline CyC_INT                  width() const               { return width_; }
    inline CyC_INT                  height() const              { return height_; }
    inline CyC_INT                  channels() const            { return channels_; }
    inline float                    min_range() const           { return min_range_; }
    inline float                    max_range() const           { return max_range_; }
    inline const bool               distortion() const          { return distortion_; } //!< is it pure pinhole model or it has radial distortion?
    inline const Eigen::Matrix3f&   K() const                   { return K_; };
    inline const cv::Mat&           cvK() const                 { return cvK_; };
    inline const cv::Mat&           cvD() const                 { return cvD_; };
    inline const Eigen::Matrix3f&   K_inv() const               { return K_inv_; };
    inline float                    fx_px() const               { return fx_px_; };
    inline float                    fy_px() const               { return fy_px_; };
    inline float                    fx_m() const                { return fx_m_; };
    inline float                    fy_m() const                { return fy_m_; };
    inline float                    cx() const                  { return cx_; };
    inline float                    cy() const                  { return cy_; };
    inline float                    sx() const                  { return sx_; };
    inline float                    sy() const                  { return sy_; };
    inline float                    D_k1() const                { return d_[0]; };
    inline float                    D_k2() const                { return d_[1]; };
    inline float                    D_p1() const                { return d_[2]; };
    inline float                    D_p2() const                { return d_[3]; };
    inline float                    D_k3() const                { return d_[4]; };
    inline float                    b() const                   { return b_; };

    void            undistortImage(const cv::Mat& raw, cv::Mat& rectified);
    Eigen::Vector2f undistort(const Eigen::Vector2f& _pt) const;
    CycPoint        undistort(const CycPoint& _pt) const;
    void            undistort(const std::vector<Eigen::Vector2f>& _pts_dist, std::vector<Eigen::Vector2f>& _pts_undist) const;
    void            undistort(const CycPoints& _pts_dist, CycPoints& _pts_undist) const;
    Eigen::Vector2f distort(const Eigen::Vector2f& _pt) const;
    void            distort(const std::vector<Eigen::Vector2f>& _pts_undist, std::vector<Eigen::Vector2f>& _pts_dist) const;

    Eigen::Vector3f normalize(const Eigen::Vector3f& _px) const;
    Eigen::Vector3f normalize(const Eigen::Vector2f& _px) const;
    Eigen::Vector3f unnormalize(const Eigen::Vector3f& _px) const;

    const bool                  inView(const Eigen::Vector2f& _pt) const;
    const bool                  inView(const cv::Point2f& _pt) const;
    const bool                  inRange(const float& _depth) const;
    Eigen::Matrix<float, 2, 3>  projectJac(const Eigen::Vector3f& xyz) const;

private:
    virtual bool loadSensorModel(const std::string& calibration_file);

private:
    bool            bIsRightCamera_;
    CyC_INT         width_, height_;
    CyC_INT         channels_;
    float           min_range_;
    float           max_range_;
    float           fx_px_, fy_px_;        // focal lenght [px] = focal length [m] / pixel size [m]
    float           fx_m_, fy_m_;          // focal lenght [m] = focal length [px] * pixel size [m]
    float           cx_, cy_;              // optical center [px]
    float           sx_, sy_;              // pixel size [m]
    bool            distortion_;           //!< is it pure pinhole model or it has radial distortion?
    float           d_[5];                 //!< distortion parameters, see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    cv::Mat         cvK_, cvD_;
    cv::Mat         undist_map1_, undist_map2_;
    bool            use_optimization_;
    Eigen::Matrix3f K_;
    Eigen::Matrix3f K_inv_;
    float           b_, pixConv_;
};

#endif /* CPinholeCameraSensorModel_H_ */
