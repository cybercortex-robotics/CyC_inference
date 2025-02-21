// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CKeypointsMatching_H_
#define CKeypointsMatching_H_

#include "CCR_TYPES.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class CKeypointsMatching
{
public:
    CKeypointsMatching();
    CKeypointsMatching(const CKeypointsMatching&) = delete;
    CKeypointsMatching(CKeypointsMatching&&) = delete;
    CKeypointsMatching operator=(const CKeypointsMatching&) = delete;
    CKeypointsMatching operator=(CKeypointsMatching&&) = delete;
    ~CKeypointsMatching() = default;

    static void ccrpoints2eigen(const CcrPoints& _ccr_pts, std::vector<Eigen::Vector2f>& _eigen_pts);

    static void eigen2ccrpoints(const std::vector<Eigen::Vector2f>& _eigen_pts, CcrPoints& _ccr_pts);

    static void match_points_by_id(const CcrPoints& _pts1, const CcrPoints& _pts2,
        CcrPoints& _matched_pts_1, CcrPoints& _matched_pts_2);

    static void match_points_by_id(const std::vector<cv::KeyPoint>& _pts1, const std::vector<cv::KeyPoint>& _pts2,
        const cv::Mat& _descriptors1, const cv::Mat& _descriptors2,
        CcrPoints& _matched_pts_1, CcrPoints& _matched_pts_2);

    static void match_points_by_id(const std::vector<cv::KeyPoint>& _pts1, const std::vector<cv::KeyPoint>& _pts2,
        std::vector<int> _matches, CcrPoints& _matched_pts_1, CcrPoints& _matched_pts_2);

    static void match_voxels_by_id(const CcrVoxels& _vxs1, const CcrVoxels& _vxs2,
        CcrVoxels& _matched_vxs_1, CcrVoxels& _matched_vxs_2);

    static void match_voxels_by_id(const CcrVoxels& _vxs1, const CcrVoxels& _vxs2, const CcrVoxels& _vxs3,
        CcrVoxels& _matched_vxs_1, CcrVoxels& _matched_vxs_2, CcrVoxels& _matched_vxs_3);

    static void match_points_by_id(const CcrPoints& _pts, const CcrVoxels& _vxs,
        CcrPoints& _matched_pts, CcrVoxels& _matched_vxs);

    static void match_points_by_id(const std::vector<cv::KeyPoint>& _pts, const CcrVoxels& _vxs,
        CcrPoints& _matched_pts, CcrVoxels& _matched_vxs);

    static bool getVoxelById(const CcrVoxels& _voxels, const CCR_INT _id, CcrVoxel& _out_voxel);

    static void get_2d_gradient(const Eigen::Vector2f& _pt1, const Eigen::Vector2f& _pt2,
        float& _magnitude, float& _orientation);

    static void get_2d_gradients(const std::vector<Eigen::Vector2f>& _pts1, const std::vector<Eigen::Vector2f>& _pts2,
        std::vector<float>& _magnitudes, std::vector<float>& _orientations);

    static void hist_2d_gradients(const std::vector<float>& _magnitudes, const std::vector<float>& _orientations,
        std::vector<int>& _magnitude_bins, std::vector<int>& _orientation_bins);

    static void plot_hist_2d_gradients(cv::Mat& _disp,
        const std::vector<float>& _magnitudes,
        const std::vector<float>& _orientations,
        const std::vector<int>& _magnitude_bins,
        const std::vector<int>& _orientation_bins);

    static void detect_checkerboard_keypts(
        const cv::Size& _checkerboard_size,
        const cv::Mat& _img1,
        const cv::Mat& _img2,
        std::vector<Eigen::Vector2f>& _out_keypts_1,
        std::vector<Eigen::Vector2f>& _out_keypts_2
    );
};

#endif /* CKeypointsMatching_H_ */
