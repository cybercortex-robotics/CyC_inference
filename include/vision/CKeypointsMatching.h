// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CKeypointsMatching_H_
#define CKeypointsMatching_H_

#include "CyC_TYPES.h"
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

    static void ccrpoints2eigen(const CycPoints& _ccr_pts, std::vector<Eigen::Vector2f>& _eigen_pts);

    static void eigen2cycpoints(const std::vector<Eigen::Vector2f>& _eigen_pts, CycPoints& _ccr_pts);

    static void match_points_by_id(const CycPoints& _pts1, const CycPoints& _pts2,
        CycPoints& _matched_pts_1, CycPoints& _matched_pts_2);

    static void match_points_by_id(const std::vector<cv::KeyPoint>& _pts1, const std::vector<cv::KeyPoint>& _pts2,
        const cv::Mat& _descriptors1, const cv::Mat& _descriptors2,
        CycPoints& _matched_pts_1, CycPoints& _matched_pts_2);

    static void match_points_by_id(const std::vector<cv::KeyPoint>& _pts1, const std::vector<cv::KeyPoint>& _pts2,
        std::vector<int> _matches, CycPoints& _matched_pts_1, CycPoints& _matched_pts_2);

    static void match_voxels_by_id(const CycVoxels& _vxs1, const CycVoxels& _vxs2,
        CycVoxels& _matched_vxs_1, CycVoxels& _matched_vxs_2);

    static void match_voxels_by_id(const CycVoxels& _vxs1, const CycVoxels& _vxs2, const CycVoxels& _vxs3,
        CycVoxels& _matched_vxs_1, CycVoxels& _matched_vxs_2, CycVoxels& _matched_vxs_3);

    static void match_points_by_id(const CycPoints& _pts, const CycVoxels& _vxs,
        CycPoints& _matched_pts, CycVoxels& _matched_vxs);

    static void match_points_by_id(const std::vector<cv::KeyPoint>& _pts, const CycVoxels& _vxs,
        CycPoints& _matched_pts, CycVoxels& _matched_vxs);

    static bool getVoxelById(const CycVoxels& _voxels, const CyC_INT _id, CycVoxel& _out_voxel);

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
