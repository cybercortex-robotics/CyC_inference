// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef COcTreeUtils_H_
#define COcTreeUtils_H_

#include "CyC_TYPES.h"
#include <octomap/octomap.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include "sensors/CPinholeCameraSensorModel.h"
#include "env/CObjectClasses.h"

class COcTreeUtils
{ 
public:
    COcTreeUtils() {};
	virtual ~COcTreeUtils() = default;

    /**
     * \brief Prints the desription of an octree
     *
     * \param _octree               Output 3D octree
     **/
    static void printOcTree(const CcrOcTree& _octree);

    /**
     * \brief Converts a depth image to an octree
     *
     * \param _psensor_model        Input camera sensor model
     * \param _img_depth_meters     Input depth image in meters
     * \param _pose                 Input pose
     * \param _poctree              Output octree
     * \param _step                 Input step for parsing the input depth image
     * \param _octree_depth_th      Input octree depth range
     * 
     * \return True if the conversion was successful
     **/
    static bool depth2octree(const CPinholeCameraSensorModel* _psensor_model,
        const cv::Mat& _img_depth_meters,
        const CPose& _pose,
        CcrOcTree* _poctree,
        const CyC_UINT& _step = 1,
        const float _octree_depth_th = 30.f,
        const float _ground_th = 0.2f,
        const float _sealing_th = 2.f,
        const CyC_INT _cls = -1,
        const Eigen::Vector3i _color = Eigen::Vector3i::Zero(),
        const float _value = 1.f);

    static bool depth2octree(const CPinholeCameraSensorModel* _psensor_model,
        const CycImage_& _rimg,
        const CPose& _pose,
        CcrOcTree* _poctree,
        const CyC_UINT& _step = 1,
        float _octree_depth_th = 30.f,
        const float _ground_th = 0.2f,
        const float _sealing_th = 2.f,
        const CyC_INT _cls = -1,
        const Eigen::Vector3i _color = Eigen::Vector3i::Zero(),
        const float _value = 1.f);

    /**
     * \brief Converts a set of ultrasonic measurements to an octree
     *
     * \param _ultrasonics          Input ultrasonics measurements
     * \param _octree               Output 3D octree
	 * \param _octree_depth_range   Input octree depth range
     * \param color                 Obstacle color
     **/
    static void ultrasonics2octree(
        const CycUltrasonics& _ultrasonics,
        const CPose& pose,
        CcrOcTree& _octree, 
		float _octree_depth_range,
        const cv::Scalar& color=cv::Scalar(0, 255, 0));

    /**
     * \brief Converts a set of 3d bounding boxes measurements to an octree
     *
     * \param _objects              Input 3D bounding boxes
     * \param _octree               Output 3D octree
     * \param _octree_depth_range   Input octree depth range
     **/
    static void bboxes3d2octree(
        const CycBBoxes3D& _objects,
        CcrOcTree& _octree,
        const Eigen::Vector3f& origin,
        float origin_yaw);

    static void trajectory2octree(
        const CycBBoxes3D& _objects,
        const std::vector<CycTrajectory>& _trajectories,
        CcrOcTree& _octree,
        const Eigen::Vector3f& origin,
        float origin_yaw);

    /**
     * \brief Converts a set of voxels to an octree
     *
     * \param _voxels       Input voxels
     * \param _octree       Output octree pointer
     * \param _cls          Input octree nodes class (used mainly for vizualization)
     * \param _value        Input octree nodes value
     * \param _rgb_color    Input octree nodes color
     **/
    static void voxels2octree(const CycVoxels& _voxels,
        CcrOcTree* _octree,
        const Eigen::Vector3i _rgb_color = Eigen::Vector3i(0, 0, 0),
        const CyC_INT _cls = -1,
        const float _value = 0);

    static void voxels2octree(const CycVoxels& _voxels,
        CcrOcTree* _octree,
        const std::vector<Eigen::Vector3i> _rgb_colors,
        const CyC_INT _cls = -1,
        const float _value = 0);

    /**
     * \brief Converts a semantic segmentation image to octree
     *
     * \param _psensor_model    Input camera sensor model
     * \param _inference_image  Input semantic segmentation image
     * \param _pose             Input transformation matrix
     * \param _depth_range      Input depth range
     * \param _octree           Output 3D octree
     **/
    static void semseg_ground2octree(const CPinholeCameraSensorModel* _psensor_model, 
        const CycImage_& _inference_image,
        const std::vector<CyC_INT> _ground_class_ids,
        const float _depth_range,
        CcrOcTree& _octree);
    
    /**
     * \brief       Copies the data from an ontree into another octree (without deleting previous data)
     *
     * \param _src  Input source octree
     * \param _dst  Output destination octree
     **/
    static void copyOctreeData(const CcrOcTree& _src, CcrOcTree& _out_dst);

    /**
     * \brief Transforms an octree environment model based on the _pose coordinates transform
     *
     * \param _octree   Input/Output octree model
     * \param _pose     Coordinates transformation pose
     **/
    static void transformOctree(CcrOcTree& _octree, const CPose& _pose);

    /**
     * \brief Converts a 3D OcTree to a 2D occupancy grid map. (version 2)
     *        The resolution of the 2D occupancy grid is the same as the OcTree resolution
     *
     *     ^ height
     *     |
     *     |
     *     |
     *     |--------->
     *   (0,0)       width
     * A* coordinates system
     *
     * \param _octree   Input 3D OcTree
     * \param _gridmap  Output 2D grid map
     * \param _size     Size of the 2D occupancy grid map as width (x) and lenght (y) (in meters)
     *
     *
     * The gridmap resolution (in pixels) will be equal to the number of cm
     * i.e. If _width is 5m and _height is 5m, the gridmap will be 500x500 px
     **/
    static void octree2gridmap(const CcrOcTree& _octree, Eigen::MatrixXi& _gridmap, const Eigen::Vector2f _size = { 10.f, 10.f });

    /**
    * \brief Projects an octree into an image according to the given sensor model
    * 
    * \param _octree        Input 3D OcTree
    * \param _sensor_model  Input camera sensor model
    * \param _dst           Output image with projected octree
    */
    static void octree2image(const CcrOcTree& _octree, const CBaseSensorModel* _sensor_model, cv::Mat& _dst);
};

#endif /* COcTreeUtils_H_ */
