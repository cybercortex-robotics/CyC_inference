// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CKeypointsMatching.h"
#include<opencv2/calib3d.hpp>

CKeypointsMatching::CKeypointsMatching()
{}

void CKeypointsMatching::ccrpoints2eigen(const CycPoints& _ccr_pts, std::vector<Eigen::Vector2f>& _eigen_pts)
{
    _eigen_pts.clear();

    for (size_t i = 0; i < _ccr_pts.size(); ++i)
        _eigen_pts.emplace_back(_ccr_pts[i].pt2d);
}

void CKeypointsMatching::eigen2cycpoints(const std::vector<Eigen::Vector2f>& _eigen_pts, CycPoints& _ccr_pts)
{
    _ccr_pts.clear();

    for (size_t i = 0; i < _eigen_pts.size(); ++i)
        _ccr_pts.emplace_back(CycPoint(_eigen_pts[i]));
}

void CKeypointsMatching::match_points_by_id(const CycPoints& _pts1, const CycPoints& _pts2,
    CycPoints& _matched_pts_1, CycPoints& _matched_pts_2)
{
    _matched_pts_1.clear();
    _matched_pts_2.clear();
    
    for (size_t i = 0; i < _pts1.size(); ++i)
    {
        for (size_t j = 0; j < _pts2.size(); ++j)
        {
            if (_pts1[i].id == _pts2[j].id && _pts1[i].id > -1)
            {
                _matched_pts_1.emplace_back(_pts1[i]);
                _matched_pts_2.emplace_back(_pts2[j]);
            }
        }
    }
}

void CKeypointsMatching::match_points_by_id(const std::vector<cv::KeyPoint>& _pts1, const std::vector<cv::KeyPoint>& _pts2,
    const cv::Mat& _descriptors1, const cv::Mat& _descriptors2,
    CycPoints& _matched_pts_1, CycPoints& _matched_pts_2)
{
    _matched_pts_1.clear();
    _matched_pts_2.clear();

    for (size_t i = 0; i < _pts1.size(); ++i)
    {
        for (size_t j = 0; j < _pts2.size(); ++j)
        {
            if (_pts1[i].class_id > -1 && (_pts1[i].class_id == _pts2[j].class_id))
            {
                _matched_pts_1.emplace_back(CycPoint(_pts1[i].pt.x, _pts1[i].pt.y, -1.f, _pts1[i].class_id, static_cast<float>(_pts1[i].octave), _descriptors1.row(static_cast<int>(i)), _pts1[i].angle));
                _matched_pts_2.emplace_back(CycPoint(_pts2[j].pt.x, _pts2[j].pt.y, -1.f, _pts2[j].class_id, static_cast<float>(_pts2[j].octave), _descriptors2.row(static_cast<int>(j)), _pts2[j].angle));
            }
        }
    }
}

void CKeypointsMatching::match_points_by_id(const std::vector<cv::KeyPoint>& _pts1, const std::vector<cv::KeyPoint>& _pts2,
    std::vector<int> _matches, CycPoints& _matched_pts_1, CycPoints& _matched_pts_2)
{
    for (size_t i = 0; i < _matches.size(); ++i)
    {
        if (_matches[i] > -1)
        {
            _matched_pts_1.emplace_back(CycPoint(_pts1[i].pt.x, _pts1[i].pt.y, static_cast<float>(_pts1[i].class_id), _pts1[i].octave));
            _matched_pts_2.emplace_back(CycPoint(_pts2[_matches[i]].pt.x, _pts2[_matches[i]].pt.y, static_cast<float>(_pts2[_matches[i]].class_id), _pts2[_matches[i]].octave));
        }
    }
}

void CKeypointsMatching::match_voxels_by_id(const CycVoxels& _vxs1, const CycVoxels& _vxs2,
    CycVoxels& _matched_vxs_1, CycVoxels& _matched_vxs_2)
{
    _matched_vxs_1.clear();
    _matched_vxs_2.clear();

    for (size_t i = 0; i < _vxs1.size(); ++i)
    {
        for (size_t j = 0; j < _vxs2.size(); ++j)
        {
            if (_vxs1[i].id == _vxs2[j].id)
            {
                _matched_vxs_1.emplace_back(_vxs1[i]);
                _matched_vxs_2.emplace_back(_vxs2[j]);
            }
        }
    }
}

void CKeypointsMatching::match_voxels_by_id(const CycVoxels& _vxs1, const CycVoxels& _vxs2, const CycVoxels& _vxs3,
    CycVoxels& _matched_vxs_1, CycVoxels& _matched_vxs_2, CycVoxels& _matched_vxs_3)
{
    _matched_vxs_1.clear();
    _matched_vxs_2.clear();
    _matched_vxs_3.clear();

    for (size_t i = 0; i < _vxs1.size(); ++i)
    {
        for (size_t j = 0; j < _vxs2.size(); ++j)
        {
            for (size_t k = 0; k < _vxs3.size(); ++k)
            {
                if (_vxs1[i].id == _vxs2[j].id)
                {
                    if (_vxs2[j].id == _vxs3[k].id)
                    {
                        _matched_vxs_1.emplace_back(_vxs1[i]);
                        _matched_vxs_2.emplace_back(_vxs2[j]);
                        _matched_vxs_3.emplace_back(_vxs3[k]);
                    }
                }
            }
        }
    }
}

void CKeypointsMatching::match_points_by_id(const CycPoints& _pts, const CycVoxels& _vxs,
    CycPoints& _matched_pts, CycVoxels& _matched_vxs)
{
    _matched_pts.clear();
    _matched_vxs.clear();

    for (size_t i = 0; i < _pts.size(); ++i)
    {
        for (size_t j = 0; j < _vxs.size(); ++j)
        {
            if (_pts[i].id == _vxs[j].id)
            {
                _matched_pts.emplace_back(_pts[i]);
                _matched_vxs.emplace_back(_vxs[j]);
            }
        }
    }
}

void CKeypointsMatching::match_points_by_id(const std::vector<cv::KeyPoint>& _pts, const CycVoxels& _vxs,
    CycPoints& _matched_pts, CycVoxels& _matched_vxs)
{
    _matched_pts.clear();
    _matched_vxs.clear();

    for (size_t i = 0; i < _pts.size(); ++i)
    {
        for (size_t j = 0; j < _vxs.size(); ++j)
        {
            if (_pts[i].class_id == _vxs[j].id)
            {
                _matched_pts.emplace_back(CycPoint(_pts[i].pt.x, _pts[i].pt.y, static_cast<float>(_pts[i].class_id), _pts[i].octave));
                _matched_vxs.emplace_back(_vxs[j]);
            }
        }
    }
}

bool CKeypointsMatching::getVoxelById(const CycVoxels& _voxels, const CyC_INT _id, CycVoxel& _out_voxel)
{
    for (const CycVoxel& vx : _voxels)
    {
        if (vx.id == _id)
        {
            _out_voxel = vx;
            return true;
        }
    }

    return false;
}

void CKeypointsMatching::get_2d_gradient(const Eigen::Vector2f& _pt1, const Eigen::Vector2f& _pt2, float& _magnitude, float& _orientation)
{
    _magnitude = sqrtf((_pt2.x() - _pt1.x()) * (_pt2.x() - _pt1.x()) + (_pt2.y() - _pt1.y()) * (_pt2.y() - _pt1.y()));
    _orientation = atan2f((_pt2.y() - _pt1.y()), (_pt2.x() - _pt1.x()));
}

void CKeypointsMatching::get_2d_gradients(const std::vector<Eigen::Vector2f>& _pts1, const std::vector<Eigen::Vector2f>& _pts2, std::vector<float>& _magnitudes, std::vector<float>& _orientations)
{
    assert(_pts1.size() != _pts2.size());
    _magnitudes.clear();
    _orientations.clear();

    for (size_t i = 0; i < _pts1.size(); ++i)
    {
        float mag, orient;
        get_2d_gradient(_pts1[i], _pts2[i], mag, orient);
        _magnitudes.emplace_back(mag);
        _orientations.emplace_back(orient);
    }
}

void CKeypointsMatching::hist_2d_gradients(const std::vector<float>& _magnitudes, const std::vector<float>& _orientations, std::vector<int>& _magnitude_bins, std::vector<int>& _orientation_bins) //float& _most_freq_mag, float& _most_freq_ori)
{
    assert(_magnitudes.size() != _orientations.size());

    _magnitude_bins.clear();
    _orientation_bins.clear();

    int num_magnitude_bins = 100;
    int num_orientation_bins = 360;

    _magnitude_bins.resize(num_magnitude_bins, 0);
    _orientation_bins.resize(num_orientation_bins, 0);

    // Normalize magnitude values and calculate magnitude bins
    float max_magnitude_val = *std::max_element(std::begin(_magnitudes), std::end(_magnitudes));
    for (size_t i = 0; i < _magnitudes.size(); ++i)
    {
        float mag_normalized = _magnitudes[i] / max_magnitude_val;
        _magnitude_bins[static_cast<size_t>(std::roundf(mag_normalized * (float)num_magnitude_bins))]++;
    }

    // Calculate orientation bins
    for (size_t i = 0; i < _orientations.size(); ++i)
        _orientation_bins[static_cast<size_t>(std::roundf(_orientations[i] * RAD2DEG) + 180.f)]++;
}

void CKeypointsMatching::plot_hist_2d_gradients(cv::Mat& _disp, const std::vector<float>& _magnitudes, const std::vector<float>& _orientations, const std::vector<int>& _magnitude_bins, const std::vector<int>& _orientation_bins)
{
    float m_debug_scaling = 1.f;
    CyC_INT m_nTextOffsetY = 28;
    cv::Scalar m_color_info = CV_RGB(100, 200, 220);
    cv::Scalar m_color_curr_pts_2d = CV_RGB(255, 255, 0);
    cv::Scalar m_color_reproj_pts_2d_pos_depth = CV_RGB(0, 255, 0);
    cv::Scalar m_color_reproj_pts_2d_neg_depth = CV_RGB(0, 0, 255);
    cv::Scalar m_color_reprojection_err = CV_RGB(255, 0, 0);

    char str[128];
    float bins_draw_scaling = 180.f;
    float text_position_scaling = 2.f;

    int num_magnitude_bins = static_cast<int>(_magnitude_bins.size());
    int num_orientation_bins = static_cast<int>(_orientation_bins.size());

    std::vector<float> normalized_magnitude_bins, normalized_orientation_bins;

    // Normalize magnitude bins
    float max_mag = static_cast<float>(*std::max_element(std::begin(_magnitude_bins), std::end(_magnitude_bins)));
    for (int i = 0; i < _magnitude_bins.size(); ++i)
        normalized_magnitude_bins.emplace_back(_magnitude_bins[i] / max_mag);

    // Normalize orientation bins
    float max_ori = static_cast<float>(*std::max_element(std::begin(_orientation_bins), std::end(_orientation_bins)));
    for (int i = 0; i < _orientation_bins.size(); ++i)
        normalized_orientation_bins.emplace_back(_orientation_bins[i] / max_ori);

    // Draw magnitudes
    cv::Mat img_hist_mags = cv::Mat::zeros(cv::Size(num_magnitude_bins, 200), CV_8UC3);
    for (int i = 0; i < normalized_magnitude_bins.size(); ++i)
    {
        cv::Point pt1(i, img_hist_mags.rows + 1);
        cv::Point pt2(i, static_cast<int>(img_hist_mags.rows + 1 - (normalized_magnitude_bins[i] * bins_draw_scaling)));
        cv::line(img_hist_mags, pt1, pt2, CV_RGB(255, 0, 0), 1);
    }
    cv::resize(img_hist_mags, img_hist_mags, cv::Size(360, 200));
    cv::resize(img_hist_mags, img_hist_mags, cv::Size(static_cast<int>(img_hist_mags.cols * text_position_scaling), static_cast<int>(img_hist_mags.rows * text_position_scaling)));
    cv::putText(img_hist_mags, "0", cv::Point(static_cast<int>(4 * text_position_scaling), static_cast<int>(12 * text_position_scaling)), cv::FONT_HERSHEY_PLAIN, 1.5, m_color_info, 2);
    float max_magnitude = *std::max_element(std::begin(_magnitudes), std::end(_magnitudes));
    snprintf(str, sizeof(str) - 1, "%f", max_magnitude);
    cv::putText(img_hist_mags, str, cv::Point(static_cast<int>(img_hist_mags.cols - (65 * text_position_scaling)), static_cast<int>(12 * text_position_scaling)), cv::FONT_HERSHEY_PLAIN, 1.5, m_color_info, 2);
    cv::resize(img_hist_mags, img_hist_mags, cv::Size(static_cast<int>(img_hist_mags.cols / text_position_scaling), static_cast<int>(img_hist_mags.rows / text_position_scaling)));

    // Draw orientations
    cv::Mat img_hist_orient = cv::Mat::zeros(cv::Size(num_orientation_bins, 200), CV_8UC3);
    for (int i = 0; i < normalized_orientation_bins.size(); ++i)
    {
        cv::Point pt1(i, img_hist_orient.rows + 1);
        cv::Point pt2(i, static_cast<int>(img_hist_orient.rows + 1 - (normalized_orientation_bins[i] * bins_draw_scaling)));
        cv::line(img_hist_orient, pt1, pt2, CV_RGB(255, 255, 0), 1);
    }


    cv::resize(img_hist_orient, img_hist_orient, cv::Size(360, 200));
    cv::resize(img_hist_orient, img_hist_orient, cv::Size(static_cast<int>(img_hist_orient.cols * text_position_scaling), static_cast<int>(img_hist_orient.rows * text_position_scaling)));
    cv::putText(img_hist_orient, "-179 [deg]", cv::Point(static_cast<int>(4 * text_position_scaling), static_cast<int>(12 * text_position_scaling)), cv::FONT_HERSHEY_PLAIN, 1.5, m_color_info, 2);
    cv::putText(img_hist_orient, "179 [deg]", cv::Point(static_cast<int>(img_hist_orient.cols - (65 * text_position_scaling)), static_cast<int>(12 * text_position_scaling)), cv::FONT_HERSHEY_PLAIN, 1.5, m_color_info, 2);
    cv::resize(img_hist_orient, img_hist_orient, cv::Size(static_cast<int>(img_hist_orient.cols / text_position_scaling), static_cast<int>(img_hist_orient.rows / text_position_scaling)));

    cv::hconcat(img_hist_mags, img_hist_orient, _disp);
}

void CKeypointsMatching::detect_checkerboard_keypts(
    const cv::Size& _checkerboard_size,
    const cv::Mat& _img1,
    const cv::Mat& _img2,
    std::vector<Eigen::Vector2f>& _out_keypts_1,
    std::vector<Eigen::Vector2f>& _out_keypts_2
)
{
    assert(!_img1.empty() && !_img2.empty());

    _out_keypts_1.clear();
    _out_keypts_2.clear();

    cv::Mat work1, work2;

    _img1.copyTo(work1);
    _img2.copyTo(work2);

    if(work1.channels() != 1)
        cv::cvtColor(work1, work1, cv::COLOR_BGR2GRAY);

    if(work2.channels() != 1)
        cv::cvtColor(work2, work2, cv::COLOR_BGR2GRAY);

    /* cv::findChessboardCorners refuses to work with Eigen::Vector2f so we use cv::Point2f. */
    std::vector<cv::Point2f> tmp1, tmp2;

    /* detect corners */
    bool ok1 = cv::findChessboardCorners(work1, _checkerboard_size, tmp1);
    bool ok2 = cv::findChessboardCorners(work2, _checkerboard_size, tmp2);

    /* check that the same number of keypoints were found for both images. */
    if( (!ok1 || !ok2) || (tmp1.size() != tmp2.size()) )
        return;

    cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 100, 0.0001);

    /* refine points based on 'criteria'. */
    cv::cornerSubPix(work1, tmp1, cv::Size(5, 5), cv::Size(-1, -1), criteria);
    cv::cornerSubPix(work2, tmp2, cv::Size(5, 5), cv::Size(-1, -1), criteria);

    for(const auto &p: tmp1)
        _out_keypts_1.emplace_back(p.x, p.y);

    for(const auto &p: tmp2)
        _out_keypts_2.emplace_back(p.x, p.y);
}
