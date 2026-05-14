// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CPinholeCameraSensorModel.h"
#include "vision/CProjectiveGeometry.h"

CPinholeCameraSensorModel::CPinholeCameraSensorModel(const std::string& calibration_file, const bool& _is_right_camera) :
    CBaseSensorModel(calibration_file),
    bIsRightCamera_(_is_right_camera)
{
    // Check if the calibration file exists
    if (!CFileUtils::FileExist(calibration_file.c_str()))
    {
        spdlog::error("{}: Calibration file '{}' does not exist.", typeid(*this).name(), calibration_file);
    }
    else
    {
        if (this->loadSensorModel(calibration_file))
            spdlog::info("{}: Calibration loaded from \"{}\"", typeid(*this).name(), calibration_file);
        else
            spdlog::warn("{}: Could not load sensor calibration from \"{}\". Sensor disabled.", typeid(*this).name(), calibration_file);
    }
}

CPinholeCameraSensorModel::~CPinholeCameraSensorModel()
{}

bool CPinholeCameraSensorModel::loadSensorModel(const std::string& calibration_file)
{
    if (fs::exists(calibration_file.c_str()))
    {
        libconfig::Config configFile;
        configFile.readFile(calibration_file.c_str());

        const libconfig::Setting& rootConfig = configFile.getRoot();
        configFile.lookupValue("image_width", width_);
        configFile.lookupValue("image_height", height_);
        configFile.lookupValue("baseline", b_);

        if (rootConfig.exists("range"))
        {
            const libconfig::Setting& range = rootConfig.lookup("range");
            min_range_ = range[0];
            max_range_ = range[1];
        }
        else
        {
            spdlog::error("Camera range parameter has to be set in '{}'. Exiting.", calibration_file);
            exit(EXIT_FAILURE);
        }

        if (!bIsRightCamera_)
        {
            const libconfig::Setting& LeftSensor = rootConfig["LeftSensor"];
            LeftSensor.lookupValue("channels", channels_);
            LeftSensor.lookupValue("focal_length_x", fx_px_);
            LeftSensor.lookupValue("focal_length_y", fy_px_);
            LeftSensor.lookupValue("optical_center_x", cx_);
            LeftSensor.lookupValue("optical_center_y", cy_);
            LeftSensor.lookupValue("pixel_size_x", sx_);
            LeftSensor.lookupValue("pixel_size_y", sy_);
            LeftSensor.lookupValue("dist_coeff_0", d_[0]);
            LeftSensor.lookupValue("dist_coeff_1", d_[1]);
            LeftSensor.lookupValue("dist_coeff_2", d_[2]);
            LeftSensor.lookupValue("dist_coeff_3", d_[3]);
            LeftSensor.lookupValue("dist_coeff_4", d_[4]);
        }
        else
        {
            const libconfig::Setting& RightSensor = rootConfig["RightSensor"];
            RightSensor.lookupValue("channels", channels_);
            RightSensor.lookupValue("focal_length_x", fx_px_);
            RightSensor.lookupValue("focal_length_y", fy_px_);
            RightSensor.lookupValue("optical_center_x", cx_);
            RightSensor.lookupValue("optical_center_y", cy_);
            RightSensor.lookupValue("pixel_size_x", sx_);
            RightSensor.lookupValue("pixel_size_y", sy_);
            RightSensor.lookupValue("dist_coeff_0", d_[0]);
            RightSensor.lookupValue("dist_coeff_1", d_[1]);
            RightSensor.lookupValue("dist_coeff_2", d_[2]);
            RightSensor.lookupValue("dist_coeff_3", d_[3]);
            RightSensor.lookupValue("dist_coeff_4", d_[4]);
        }

        init();

        return true;
    }
    else
    {
        return false;
    }
}

void CPinholeCameraSensorModel::init()
{
    if (fabs(d_[0]) > 0.0000001)
        distortion_ = true;
    else
        distortion_ = false;

    undist_map1_ = cv::Mat(height_, width_, CV_16SC2);
    undist_map2_ = cv::Mat(height_, width_, CV_16SC2);
    use_optimization_ = false;

    fx_m_ = fx_px_ * sx_;
    fy_m_ = fy_px_ * sy_;

    cvK_ = (cv::Mat_<float>(3, 3) << fx_px_, 0.0, cx_, 0.0, fy_px_, cy_, 0.0, 0.0, 1.0);
    cvD_ = (cv::Mat_<float>(1, 5) << d_[0], d_[1], d_[2], d_[3], d_[4]);
    cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<float>::eye(3, 3), cvK_, cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_);
    K_ << fx_px_, 0.0, cx_, 0.0, fy_px_, cy_, 0.0, 0.0, 1.0;
    K_inv_ = K_.inverse();
}

Eigen::Vector3f CPinholeCameraSensorModel::sensor2world(const float& u, const float& v) const
{
    Eigen::Vector3f xyz;

    if (!distortion_)
    {
        xyz[0] = (u - cx_) / fx_px_;
        xyz[1] = (v - cy_) / fy_px_;
        xyz[2] = 1.0;
    }
    else
    {
        cv::Point2f uv(u, v), px;
        const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
        cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
        cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
        xyz[0] = px.x;
        xyz[1] = px.y;
        xyz[2] = 1.0;
    }

    return xyz.normalized();
}

Eigen::Vector3f CPinholeCameraSensorModel::sensor2world(const Eigen::Vector2f& uv) const
{
    return sensor2world(uv[0], uv[1]);
}

Eigen::Vector2f CPinholeCameraSensorModel::world2sensor(const Eigen::Vector2f& uv) const
{
    Eigen::Vector2f px;

    if (!distortion_)
    {
        px[0] = fx_px_ * uv[0] + cx_;
        px[1] = fy_px_ * uv[1] + cy_;
    }
    else
    {
        float x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
        x = uv[0];
        y = uv[1];
        r2 = x * x + y * y;
        r4 = r2 * r2;
        r6 = r4 * r2;
        a1 = 2 * x * y;
        a2 = r2 + 2 * x * x;
        a3 = r2 + 2 * y * y;
        cdist = 1 + d_[0] * r2 + d_[1] * r4 + d_[4] * r6;
        xd = x * cdist + d_[2] * a1 + d_[3] * a2;
        yd = y * cdist + d_[2] * a3 + d_[3] * a1;
        px[0] = xd * fx_px_ + cx_;
        px[1] = yd * fy_px_ + cy_;
    }

    return px;
}

Eigen::Vector2f CPinholeCameraSensorModel::world2sensor(const Eigen::Vector3f& xyz) const
{
    return Eigen::Vector2f{
        fx_px_ * (xyz[0] / xyz[2]) + cx_,
        fy_px_ * (xyz[1] / xyz[2]) + cy_
    };
}

Eigen::Vector2f CPinholeCameraSensorModel::world2sensor(const Eigen::Vector4f& xyz) const
{
    return Eigen::Vector2f{
        fx_px_ * (xyz[0] / xyz[2]) + cx_,
        fy_px_ * (xyz[1] / xyz[2]) + cy_
    };
}

// project a disparity point to the 3D space
Eigen::Vector3f CPinholeCameraSensorModel::depth2world(const float& u_d, const float& v_d, const float& depth) const
{
    Eigen::Vector3f xyz;

    if ((std::abs(fx_px_) < std::numeric_limits<float>::epsilon()) || (std::abs(fy_px_) < std::numeric_limits<float>::epsilon()) || (std::abs(depth) < std::numeric_limits<float>::epsilon()))
    {
        spdlog::warn("{}: Float division by zero in CPinholeCameraSensorModel::disparity2world(..). Projection not performed. Skip operation! ", typeid(*this).name());
    }
    else
    {
        //convert depth from milimeters to meters
        xyz[2] = depth / 1000; // [m]

        xyz[0] = (u_d - cx_) * xyz[2] / fx_px_;
        xyz[1] = (v_d - cy_) * xyz[2] / fy_px_;
    }

    return xyz;
}

void CPinholeCameraSensorModel::undistortImage(const cv::Mat& raw, cv::Mat& rectified)
{
    if (distortion_)
        cv::remap(raw, rectified, undist_map1_, undist_map2_, cv::INTER_LINEAR);
    else
        rectified = raw.clone();
}

Eigen::Vector2f CPinholeCameraSensorModel::distort(const Eigen::Vector2f& px) const
{
    return Eigen::Vector2f{
        px[0] * fx_px_ + cx_,
        px[1] * fy_px_ + cy_
    };
}

void CPinholeCameraSensorModel::distort(const std::vector<Eigen::Vector2f>& _pts_undist, std::vector<Eigen::Vector2f>& _pts_dist) const
{
    _pts_dist.clear();
    for (size_t i = 0; i < _pts_undist.size(); ++i)
        _pts_dist.emplace_back(this->distort(_pts_undist[i]));
}

Eigen::Vector3f CPinholeCameraSensorModel::normalize(const Eigen::Vector3f& _px) const
{
    //return K_inv_ * _px;
    Eigen::Vector3f norm = K_inv_ * _px; // distorted normalized
    Eigen::Vector2f undist = CProjectiveGeometry::undistort(this, norm.head<2>());
    return { undist.x(), undist.y(), 1.f };
}

Eigen::Vector3f CPinholeCameraSensorModel::normalize(const Eigen::Vector2f& _px) const 
{
    Eigen::Vector3f hom(_px.x(), _px.y(), 1.f);
    return normalize(hom);
}

Eigen::Vector3f CPinholeCameraSensorModel::unnormalize(const Eigen::Vector3f& _px) const
{
    Eigen::Vector3f px = _px;
    if (_px.z() != 1.f)
        px = _px / _px.z();
    Eigen::Vector2f dist = distort(px.head<2>());
    return K_ * Eigen::Vector3f(dist.x(), dist.y(), 1.f);
}

Eigen::Matrix<float, 2, 3> CPinholeCameraSensorModel::projectJac(const Eigen::Vector3f& _xyz) const
{
    Eigen::Matrix<float, 2, 3> Jac;
    Jac(0, 0) = fx_px_ / _xyz[2];
    Jac(0, 1) = 0.f;
    Jac(0, 2) = -fx_px_ * _xyz[0] / (_xyz[2] * _xyz[2]);
    Jac(1, 0) = 0.f;
    Jac(1, 1) = fy_px_ / _xyz[2];
    Jac(1, 2) = -fy_px_ * _xyz[1] / (_xyz[2] * _xyz[2]);

    return Jac;
}

const bool CPinholeCameraSensorModel::inView(const Eigen::Vector2f& _pt) const
{
    if (_pt.x() >= 0.f && _pt.x() < width_ && _pt.y() > 0.f && _pt.y() < height_)
        return true;
    else
        return false;
}

const bool CPinholeCameraSensorModel::inView(const cv::Point2f& _pt) const
{
    return inView(Eigen::Vector2f(_pt.x, _pt.y));
}

const bool CPinholeCameraSensorModel::inRange(const float& _depth) const
{
    if (_depth >= min_range_ && _depth <= max_range_)
        return true;
    else
        return false;
}
