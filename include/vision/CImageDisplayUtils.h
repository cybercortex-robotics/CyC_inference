// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CIMAGEDISPLAYUTILS_H_
#define CIMAGEDISPLAYUTILS_H_

#include "CCR_TYPES.h"
#include "env/CQuaternion.h"
#include "CBaseCcrFilter.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "math/CPolynomialFitting.h"
#include "os/CTimer.h"
#include "sensors/CPinholeCameraSensorModel.h"
#include "sensors/CImuSensorModel.h"
#include "vision/CProjectiveGeometry.h"
#include "vision/CTriangulation.h"
#include "vision/CKeypointsMatching.h"
#include "env/CObjectClasses.h"

class CImageDisplayUtils
{
public:
    CImageDisplayUtils();
    virtual ~CImageDisplayUtils();

    // 3D pose drawing functions
    static void drawPoseCamView(const cv::Mat& rmat, 
        const cv::Mat& tvec,
        const cv::Mat& K,
        const cv::Mat& D,
        cv::Mat& dst,
        const float& length = 1,
        const std::string& label = "");

    static void drawPoseCamView(const Eigen::Affine3f& T,
        const cv::Mat& K,
        const cv::Mat& D,
        cv::Mat& dst,
        const float& length = 1,
        const std::string& label = "");

    static void drawPoseCamView(const CPose& pose,
        const cv::Mat& K,
        const cv::Mat& D,
        cv::Mat& dst,
        const float& length = 1,
        const std::string& label = "");

	/**
	 * \brief Draws the detected and tracked objects
	 **/
    static void drawObjects(cv::Mat& dst, const CcrRois2D& objects, std::unordered_map<CCR_INT, std::string>& obj_classes_map, CcrDatablockKey key = {-1, -1});


    /*
     * \brief Draw semantic segmentation image
     */
    static void drawSemanticSegmentation(cv::Mat& dst, const CcrImage_& sem_obj, std::vector<CCR_UINT> classes = std::vector<CCR_UINT>({}));

    /*
     * \brief Draw semantic segmentation image (Panoptic) 
     */
    static void drawPanoSemSeg(cv::Mat& dst, const CcrImage_& sem_obj);

    /**
     * \brief Draws the lanes model
     **/
    static void drawLanes(cv::Mat& dst, const CcrLanesModel& lanes_model);
    static void drawLanes(cv::Mat& dst, const CcrLanesModel& lanes_model, const CPinholeCameraSensorModel& camera_model);

    /**
     * \brief Draws 2D points
     **/
    static void drawPoints(cv::Mat& dst, const CcrPoints& pts, CCR_INT &last_track_id);

    static void drawPoints(cv::Mat& dst, const CcrPoints& _pts, const cv::Scalar& _color);

    static void drawTrackedPoints(cv::Mat& dst, const CcrPoints& pts_curr, const CcrPoints& pts_prev);

    /**
     * \brief Draws 3D voxels
     **/
    static void drawVoxels(cv::Mat& dst, const CcrVoxels& voxels, const CPinholeCameraSensorModel& camera_model, const cv::Scalar& _color = CV_RGB(255, 114, 118));

    /**
     * \brief Draws a vehicle control input onto a given image
     **/
    static void drawVehicleControlInput(cv::Mat& dst, const CcrControlInput& control_input, const CPinholeCameraSensorModel& camera_model);

    /**
     * \brief Compute and draw the epipolar lines in two images
     *      associated to each other by a fundamental matrix
     *
     * \param title     Title of the window to display
     * \param F         Fundamental matrix
     * \param img1      First image
     * \param img2      Second image
     * \param points1   Set of points in the first image
     * \param points2   Set of points in the second image matching to the first set
     * \param inlierDistance      Points with a high distance to the epipolar lines are
     *                not displayed. If it is negative, all points are displayed
     **/
    static void drawEpipolarLines(const cv::Mat F,
        const cv::Mat& img1, 
        const cv::Mat& img2,
        const std::vector<cv::Point2f>& points1,
        const std::vector<cv::Point2f>& points2,
        cv::Mat& dst,
        const bool bDrawLeftEpilines = true,
        const bool bDrawRightEpilines = true,
        const float inlierDistance = -1);

    /**
     * \brief Draws the local environment around a vehicle in the vehicle's x-y coordinates system (top view)
     *
     * \param _vehicle_state    State of the vehicle (x, y, velocity, yaw)
     * \param _disp_ptr         display image
     **/
    static void drawLocalEgoVehicleEnvironment(const CcrState& _vehicle_state,
        cv::Mat* _disp_ptr);

    /**
     * \brief Draws a surface grid projected in a given coordinates system
     *
     **/
    static float distancePointLine(const cv::Point2f point, const cv::Vec3f& line);

    /**
     * \brief Concatenates the vector of images
     *
     * \param images            Input images vector (images have to have the same size)
     * \param cols              Number of images to be concatenated
     * \param min_gap_size      Gap between images
     **/
    static cv::Mat concatImages(std::vector<cv::Mat> & images, int cols, int min_gap_size);

    /**
     * \brief Concatenates horizontally the vectors of temporal images sequences
     *
     * \param images    Input images as N vectors, where N is the number of image streams (images have to have the same size)
     **/
    static cv::Mat concatTemporalImagesHorizontal(std::vector<std::vector<cv::Mat>>& images);

    /**
     * \brief Concatenates vertically the vectors of temporal images sequences
     *
     * \param images    Input images as N vectors, where N is the number of image streams (images have to have the same size)
     **/
    static cv::Mat concatTemporalImagesVertical(std::vector<std::vector<cv::Mat>>& images);

    static cv::Point2f rot(
        const cv::Point2f& pt,
        float theta);

    static cv::Point2f rot_t(
        const cv::Point2f& pt,
        const cv::Point2f& orig,
        float theta);
    
    /**
     * \brief Draw visual representation of ultrasonic sensors
     *
     * \param vehicleState      Current vehicle state
     * \param uss               Ultrasonic sensor array
     * \param num_us_front      Number of front US
     * \param num_us_rear       Number of rear US
     **/
    static cv::Mat ultrasonicsToImage(const CcrState& vehicleState, const CcrUltrasonics& uss, CCR_UINT num_us_front, CCR_UINT num_us_rear);

    /**
    * \brief Concatenate image sequences of ultrasonic representations
    *
    * \param usImages      Vector of ultrasonic predictions images
    **/
    static cv::Mat concatTemporalUltrasonicRepresentation(const std::vector<cv::Mat>& usImages);
    
    static void printTimestamp(cv::Mat& _dst, 
        const CCR_TIME_UNIT& _ts);
    
    /*static void printDatastreamInfo(cv::Mat& _dst,
        const CCR_TIME_UNIT& _ts,
        CBaseCcrFilter* _filter = nullptr);*/

    static void draw_correspondences(const cv::Mat& _img1, 
        const cv::Mat& _img2,
        cv::Mat& _dst,
        const std::vector<Eigen::Vector2f>& _pts1,
        const std::vector<Eigen::Vector2f>& _pts2,
        const CCR_INT& _line_thickness = 1);

    static void draw_correspondences(const cv::Mat& _img1, const cv::Mat& _img2,
        cv::Mat& _dst,
        const CcrPoints& _pts1, const CcrPoints& _pts2, 
        const CCR_INT& _line_thickness = 1,
        const std::vector<CCR_INT>& _matches = std::vector<CCR_INT>(NULL));

    // _format is either euler of quaternion
    static void printT(cv::Mat& _img, std::string _text, 
        const Eigen::Matrix4f _T, const CCR_INT _position, 
        const cv::Scalar _color = cv::Scalar(0, 0 ,0),
        const double _font_scale = 1.,
        const CCR_INT _thickness = 1,
        const std::string _format = "euler");

    static void drawLine(cv::Mat& _img, const Eigen::Vector3f& _line_coeffs, const cv::Scalar& _color = CV_RGB(255, 255, 0));
    static void drawLines(cv::Mat& _img, const std::vector<Eigen::Vector3f>& _lines_coeffs, const cv::Scalar& _color = CV_RGB(255, 255, 0));

    /**
    * \brief Resizes an image, while keeping its aspect ration
    *
    * \param _input_img Input image
    * \param _dst_size  Destination image size
    * \param _bgcolor   Background color
    * \return           Resized image
    **/
    static cv::Mat resizeKeepAspectRatio(const cv::Mat& _input_img, const cv::Size& _dst_size, const cv::Scalar& _bgcolor = cv::Scalar(0, 0, 0));

    static void drawSlam(const cv::Mat& _img,
        cv::Mat& _out_dst,
        const CPinholeCameraSensorModel* _pSensorModel,
        const CPose& _abs_cam_pose_W,
        const CPose& _relative_cam_pose_C,
        const CcrPoints& prev_inliers_pts,
        const CcrPoints& curr_inliers_pts,
        const CcrVoxels& _voxels_prev_C,
        const CcrVoxels& _voxels_curr_C,
        const float& _scale_factor,
        const std::vector<CCR_INT>& _scale_factor_samples_1,
        const std::vector<CCR_INT>& _scale_factor_samples_2,
        const bool _draw_epi_projections = false,
        const CCcrCache* _preintegrated_imu_hist = nullptr);

    static void drawSlam(const cv::Mat& _img,
        cv::Mat& _out_dst,
        const CPinholeCameraSensorModel* _pSensorModel,
        const CcrSlam& _slam_data,
        const bool _draw_epi_projections = false,
        const CCcrCache* _preintegrated_imu_hist = nullptr);

    static void drawSlam(const CcrImage_& _rimg,
        cv::Mat& _out_dst,
        const CPinholeCameraSensorModel* _pSensorModel,
        const CcrSlam& _slam_data,
        const bool _draw_epi_projections = false,
        const CCcrCache* _preintegrated_imu_hist = nullptr);

    static void draw_slam_frame(cv::Mat& _out_dst,
        const CPinholeCameraSensorModel* _pCamSensorModel,
        const CcrSlam& _slam_data);

    static void drawScaleFactor(cv::Mat& _out_dst,
        const CPinholeCameraSensorModel* _pSensorModel,
        const CPose& _relative_pose_C,
        const CcrVoxels& _voxels_prev_corresp,
        const CcrVoxels& _voxels_curr_corresp,
        const std::vector<CCR_INT>& _samples_1_ids,
        const std::vector<CCR_INT>& _samples_2_ids);

    static void drawPreintegratedImu(cv::Mat& _out_dst, const CCcrCache* _preintegrated_imu_hist = nullptr);

    static void draw_slam_grid(cv::Mat& _out_dst, const CPinholeCameraSensorModel* _pCamSensorModel);

    static void draw_slam(cv::Mat& _out_dst,
        const CPinholeCameraSensorModel* _pCamSensorModel,
        const CImuSensorModel* _pImuSensorModel,
        const CcrSlam& _slam_data,
        const float& _bew_scale,
        const bool& _absolute_coord = false,
        const bool& _draw_curr_frame = true);

    static void draw_pose(cv::Mat& _out_dst, const float& _x, const float& _y, const float& _yaw,
        const float& _scale, const cv::Scalar& _color = color::white, const CCR_INT& _line_thickness = 1);

private:
    static std::unique_ptr<CCcrCache>   m_pSlamDispCache;
    static std::vector<Eigen::Vector3f> m_SlamTrajectory;
};

#endif /* CIMAGEDISPLAYUTILS_H_ */
