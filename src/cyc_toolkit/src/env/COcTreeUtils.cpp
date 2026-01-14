// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "COcTreeUtils.h"
#include <exception>
#include <numeric>
#include <os/CTimer.h>
#include <sensors/CPinholeCameraSensorModel.h>
#include <vision/CDepthImageProcessing.h>
#include <vision/CProjectiveGeometry.h>
#include "CConfigParameters.h"

void COcTreeUtils::printOcTree(const CCycOcTree& _octree)
{
    double sizeX, sizeY, sizeZ;
    _octree.getMetricSize(sizeX, sizeY, sizeZ);

    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    _octree.getMetricMin(minX, minY, minZ);
    _octree.getMetricMax(maxX, maxY, maxZ);

    std::cout << std::endl << "*** OcTree description ***" << std::endl;
    std::cout << "- Leaf nodes:\t" << _octree.getNumLeafNodes() << std::endl;
    std::cout << "- Depth:\t" << _octree.getTreeDepth() << std::endl;
    std::cout << "- Resolution:\t" << _octree.getResolution() << "m" << std::endl;
    std::cout << "- Volume:\t" << sizeX * sizeY * sizeZ << " m^3" << std::endl;
    std::cout << "- Size (x, y, z):\t\t" << sizeX << "m, " << sizeY << "m, " << sizeZ << "m" << std::endl;
    std::cout << "- Min bounding box point:\t" << minX << "m, " << minY << "m, " << minZ << "m" << std::endl;
    std::cout << "- Max bounding box point:\t" << maxX << "m, " << maxY << "m, " << maxZ << "m" << std::endl;
    std::cout << std::endl;
}

bool COcTreeUtils::depth2octree(const CPinholeCameraSensorModel* _psensor_model, const cv::Mat& _img_depth_meters, const CPose& _pose, 
    CCycOcTree* _poctree, const CyC_UINT& _step, const float _octree_depth_th, const float _ground_th,
    const float _sealing_th, const CyC_INT _cls, const Eigen::Vector3i _color, const float _value)
{
    if (_img_depth_meters.empty() || _img_depth_meters.type() != CV_32F)
        return false;

    for (CyC_INT i = 0; i < _img_depth_meters.rows; i = i + _step)
    {
        for (CyC_INT j = 0; j < _img_depth_meters.cols; j = j + _step)
        {
            Eigen::Vector4f vx_C;
            vx_C.z() = _img_depth_meters.at<float>(i, j);

            if (vx_C.z() > 0.001f && vx_C.z() < _octree_depth_th)
            {
                vx_C.x() = ((float)j - _psensor_model->cx()) * vx_C.z() / _psensor_model->fx_px();
                vx_C.y() = ((float)i - _psensor_model->cy()) * vx_C.z() / _psensor_model->fy_px();
                vx_C.w() = 1.f;

                Eigen::Vector4f vx_W = _pose.transform() * vx_C;

                // Thresholding the ground floor and sealing
                if (vx_W.z() >= _ground_th && vx_W.z() < _sealing_th)
                {
                    if (fabsf(vx_W.x()) < 300.f && fabsf(vx_W.y()) < 300.f)
                    {
                        auto* node = _poctree->updateNode(vx_W.x(), vx_W.y(), vx_W.z(), true, true);
                        node->setObjectClass(_cls);
                        node->setValue(_value);
                        node->setColor(_color[0], _color[1], _color[2]);
                    }
                }
            }
        }
    }

    return true;
}

bool COcTreeUtils::depth2octree(const CPinholeCameraSensorModel* _psensor_model, const CycImage_& _rimg, const CPose& _pose,
    CCycOcTree* _poctree, const CyC_UINT& _step, const float _octree_depth_th, const float _ground_th,
    const float _sealing_th, const CyC_INT _cls, const Eigen::Vector3i _color, const float _value)
{
    cv::Mat cv_img_depth = cv::Mat(_rimg.nRows, _rimg.nCols, _rimg.nType2, _rimg.pData2);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    return depth2octree(_psensor_model, cv_img_depth, _pose, _poctree, _step, _octree_depth_th, _ground_th, _sealing_th, _cls, _color, _value);
}

void COcTreeUtils::voxels2octree(const CycVoxels& _voxels,
    CCycOcTree* _octree,
    const Eigen::Vector3i _rgb_color,
    const CyC_INT _cls,
    const float _value)
{
    for (const auto& voxel : _voxels)
    {
        if (fabsf(voxel.pt3d.x()) < 300.f && fabsf(voxel.pt3d.y()) < 300.f && fabsf(voxel.pt3d.z()) < 300.f)
        {
            auto* node = _octree->updateNode(voxel.pt3d.x(), voxel.pt3d.y(), voxel.pt3d.z(), true, true);
            node->setObjectClass(_cls);
            node->setValue(_value);
            node->setColor(_rgb_color[0], _rgb_color[1], _rgb_color[2]);
        }
    }
}

void COcTreeUtils::voxels2octree(const CycVoxels& _voxels,
    CCycOcTree* _octree,
    const std::vector<Eigen::Vector3i> _rgb_colors,
    const CyC_INT _cls,
    const float _value)
{
    for (size_t i = 0; i < _voxels.size(); ++i)
    {
        CycVoxel voxel = _voxels[i];

        if (fabsf(voxel.pt3d.x()) < 300.f && fabsf(voxel.pt3d.y()) < 300.f && fabsf(voxel.pt3d.z()) < 300.f)
        {
            auto* node = _octree->updateNode(voxel.pt3d.x(), voxel.pt3d.y(), voxel.pt3d.z(), true, true);
            node->setObjectClass(_cls);
            node->setValue(_value);
            node->setColor(_rgb_colors[i][0], _rgb_colors[i][1], _rgb_colors[i][2]);
        }
    }
}

void COcTreeUtils::ultrasonics2octree(const CycUltrasonics& _ultrasonics, 
    const CPose& pose,
    CCycOcTree& _octree, 
    const float _octree_depth_range,
    const cv::Scalar& color)
{
    auto max_range = 32.f;
    
    /* Find points determined by Ultrasonic range values */
    for (const auto& us : _ultrasonics)
    {
        /* Consider obstacles just the points which are less than the maximum US range */
        const auto X = us.pose.translation_3x1().x() + us.range * cosf(us.pose.rotation_euler().z());
        const auto Y = us.pose.translation_3x1().y() + us.range * sinf(us.pose.rotation_euler().z());

        /* Consider ultrasonic points at 0.45m, due to thresholding lower values (lower height values are considered on the floor) */
        const auto Z = 0.45f;

        Eigen::Vector4f before_transform(X, Y, Z, 1.F);
        Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.F);
        after_transform = pose.transform() * before_transform;

        auto* node = _octree.updateNode(after_transform.x(), after_transform.y(), after_transform.z(), true, true);
        node->setValue(CObjectClasses::ULTRASONICS);
    }
}


static void advance_state(CycState& state, float dt)
{
    state.x_hat(0) += state.x_hat(2) * cosf(state.x_hat(3)) * dt;
    state.x_hat(1) += state.x_hat(2) * sinf(state.x_hat(3)) * dt;
}


void COcTreeUtils::bboxes3d2octree(const CycBBoxes3D& _objects, CCycOcTree& _octree, const Eigen::Vector3f& origin, float origin_yaw)
{
    const octomap::point3d octree_origin(0.F, 0.F, 0.F);
    const Eigen::AngleAxisf Rz_veh(-origin_yaw, Eigen::Vector3f::UnitZ());
     
    const float res = _octree.getResolution();

    for (const auto& obj : _objects)
    {
        const float begin_x = -0.5F * obj.depth;
        const float begin_y = -0.5F * obj.width;
        const float begin_z = -0.5F * obj.height;

        const float end_x = -1.F * begin_x;
        const float end_y = -1.F * begin_y;
        const float end_z = -1.F * begin_z;

        const size_t approx_num_pts =
                (size_t)ceil((end_x - begin_x) / res) *
                (size_t)ceil((end_y - begin_y) / res) *
                (size_t)ceil((end_z - begin_z) / res);

        octomap::Pointcloud pcl;
        pcl.reserve(approx_num_pts + 10);

        for (float x = begin_x; x <= end_x; x += res)
        {
            for (float y = begin_y; y <= end_y; y += res)
            {
                for (float z = begin_z; z <= end_z; z += res)
                {
                    pcl.push_back(x, y, z);
                }
            }
        }

        const Eigen::Vector3f t = obj.origin.translation_3x1() - origin;

        const octomap::pose6d p{ t[0], t[1], t[2], 0., 0., obj.origin.rotation_euler()[2] };
        pcl.transform(p);
        pcl.rotate(0., 0., -origin_yaw);

        //const auto& color = color::colormap[(obj.cls < sizeof(color::colormap)) ? obj.cls : 0];
        for (const octomath::Vector3& pt : pcl)
        {
            auto* node = _octree.updateNode(pt.x(), pt.y(), pt.z(), false, false);
            
            if (node != nullptr)
            {
                node->setValue(obj.cls);
                node->setObjectClass(obj.cls);
                node->setLogOdds(100.f);
            }
        }
    }
}

void COcTreeUtils::trajectory2octree(const CycBBoxes3D& _objects,
    const std::vector<CycTrajectory>& _trajectories,
    CCycOcTree& _octree,
    const Eigen::Vector3f& origin,
    float origin_yaw)
{
    const octomap::point3d octree_origin(0.F, 0.F, 0.F);
    const Eigen::AngleAxisf Rz_veh(-origin_yaw, Eigen::Vector3f::UnitZ());

    static const cv::Scalar colors[] = { // RGB!
            { 0, 0, 0 }, // unknown
            { 252, 186, 3 }, // human
            { 156, 149, 132 }, // static object
            { 122, 196, 120 }, // movable object
            { 227, 118, 211 }, // animal
            { 118, 147, 227 } // vehicle
    };

    for (size_t i = 0; i < _objects.size(); ++i)
    {
        const CycBBox3D& obj = _objects[i];
        const CycTrajectory& traj = _trajectories[i];
        
        const Eigen::AngleAxisf Rz_obj(obj.origin.rotation_euler()[2], Eigen::Vector3f::UnitZ());

        Eigen::MatrixXf traj_pts(3, traj.size());
        for (size_t j = 0; j < traj.size(); ++j)
        {
            traj_pts.col(j) << traj[j][0], traj[j][1], 0.F;

            // translate points relative to vehicle position
            traj_pts.col(j) -= origin;

            // rotate the points back to world
            traj_pts.col(j) = Rz_veh.matrix() * traj_pts.col(j);
        }

        const auto& color = colors[(obj.cls < sizeof(colors)) ? obj.cls : 0];

        const float res = _octree.getResolution();
        for (Eigen::Index j = 0; j < traj_pts.cols(); ++j)
        {
            auto* node = _octree.updateNode(traj_pts.col(j)[0], traj_pts.col(j)[1], 0.F, true, true);
            node->setValue(CObjectClasses::UNDEFINED);
        }
    }
}

std::vector<size_t> statisticalOutlierRemoval(const octomap::Pointcloud& pcl, size_t numClosestPoints, float alpha)
{
    if (numClosestPoints > pcl.size())
        return {};

    Eigen::MatrixXf distMat = Eigen::MatrixXf::Zero(pcl.size(), pcl.size());
    for (size_t i = 0; i < pcl.size(); ++i)
    {
        for (size_t j = (i+1); j < pcl.size(); ++j)
        {
            distMat(i, j) = (float)pcl[i].distanceXY(pcl[j]);
            distMat(j, i) = distMat(i, j);
        }
    }

    // Compute average distance for each point
    std::vector<float> avgDist(pcl.size());
    for (size_t i = 0; i < pcl.size(); ++i)
    {
        // Initialize the closest numClosestPoints distances to the
        // first numClosestPoints in the pointcloud
        std::vector<float> closestDist(numClosestPoints);
        for (size_t j = 0; j < numClosestPoints; ++j)
        {
            closestDist[j] = distMat(i, j);
        }

        // Sort the distances since this simplifies calculations
        std::sort(closestDist.begin(), closestDist.end());

        // Compute the closest numClosestPoints distances
        for (size_t j = 0; j < pcl.size(); ++j)
        {
            // Get the distance to the current point
            const float dist = distMat(i, j);
            for (size_t k = 0; k < numClosestPoints; ++k)
            {
                if (dist < closestDist[k])
                {
                    // Push everything back in the vector, since
                    // everything is sorted
                    for (size_t l = (numClosestPoints - 1); l > k; --l)
                    {
                        closestDist[l] = closestDist[l - 1];
                    }

                    // "Add" value in the vector
                    closestDist[k] = dist;
                    break;
                }
            }
        }

        // Compute sum, but ignore the first value since that is the distance to itself
        const float sum = std::accumulate(closestDist.begin() + 1, closestDist.end(), 0.F);
        avgDist[i] = sum / (numClosestPoints - 1);
    }

    const float sum = std::accumulate(avgDist.begin(), avgDist.end(), 0.F);
    const float mean = sum / pcl.size();

    float stdDevSum = 0.F;
    for (const auto v : avgDist)
    {
        stdDevSum += powf(v - mean, 2.F);
    }

    const float stdDev = sqrtf(stdDevSum / pcl.size());
    const float T = mean + alpha * stdDev;

    std::vector<size_t> outliers;
    for (size_t i = 0; i < pcl.size(); ++i)
    {
        if (avgDist[i] > T)
        {
            outliers.push_back(i);
        }
    }

    return outliers;
}

void COcTreeUtils::semseg_ground2octree(const CPinholeCameraSensorModel* _psensor_model, const CycImage_& _inference_image, const std::vector<CyC_INT> _ground_class_ids, const float _depth_range, CCycOcTree& _octree)
{
    cv::Mat img_semseg(_inference_image.nRows, _inference_image.nCols, _inference_image.nType1);
    img_semseg.data = static_cast<uchar*>(_inference_image.pData1);

    if (!_inference_image.empty2())
    {
        /*cv::Mat depth_semseg(_inference_image.nRows, _inference_image.nCols, _inference_image.nType2, _inference_image.pData2);
        //cv::Mat decoded_depth = CDepthImageProcessing::image2depth(depth_semseg);
        spdlog::info("COcTreeUtils depth: {} -- {}", depth_semseg.type(), depth_semseg.empty());

        CycVoxels voxels;
        std::vector<Eigen::Vector3i> colors;
        //CDepthImageProcessing::depth2voxels(_psensor_model, depth_semseg, voxels, 16.f, 5, img_semseg, colors);

        for (const auto& vx : voxels)
        {
            auto* node = _octree.updateNode(vx.pt3d.x(), vx.pt3d.y(), vx.pt3d.z(), true, true);
            node->setObjectClass(1);
            node->setColor(0, 0, 0);
            node->setValue(octomap::logodds(0.7));
        }*/
    }

    cv::Mat gray_image;
    if (img_semseg.channels() > 1)
        cv::cvtColor(img_semseg, gray_image, cv::COLOR_BGR2GRAY);
    else
        gray_image = img_semseg;

    const float Y = _psensor_model->extrinsics().translation_3x1().z();

    for (CyC_INT i = 0; i < gray_image.cols; ++i)
    {
        for (CyC_INT j = 0; j < gray_image.rows; ++j)
        {
            // Check if the segmented class is in the vector of traversable classes
            CyC_INT nGroundID = -1;
            for (const auto& class_id : _ground_class_ids)
            {
                CyC_INT px = (CyC_INT)gray_image.at<unsigned char>(j, i);
                if (px == class_id)
                {
                    nGroundID = class_id;
                    break;
                }
            }

            if (nGroundID >= 0)
            {
                const cv::Point2f coord = cv::Point2f(i, j);
                
                // Calculate 3D coordinates in camera frame
                if ((coord.y - _psensor_model->cy()) != 0)
                {
                    const float Z = (_psensor_model->fy_px() * Y) / (coord.y - _psensor_model->cy());
                    const float X = (Z * (coord.x - _psensor_model->cx())) / _psensor_model->fx_px();

                    if (Z > 0.f && Z <= _depth_range)
                    {
                        // Calculate 3D coordinates in vehicle frame
                        Eigen::Vector4f pt3dCam{ X, Y, Z, 1.F };
                        Eigen::Vector4f after_transform = _psensor_model->extrinsics().transform() * pt3dCam;

                        auto* node = _octree.updateNode(after_transform.x(), after_transform.y(), after_transform.z(), true, true);
                        node->setObjectClass(nGroundID);
                        node->setValue(octomap::logodds(0.7));
                    }
                }
            }
        }
    }
}

void COcTreeUtils::copyOctreeData(const CCycOcTree& _src, CCycOcTree& _out_dst)
{
    for (auto it = _src.begin_leafs(), end = _src.end_leafs(); it != end; ++it)
    {
        auto* node = _out_dst.updateNode(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z(), true);
        node->setColor(it->getColor());
        node->setValue(it->getValue());
        node->setObjectClass(it->getObjectClass());
    }
}

void COcTreeUtils::transformOctree(CCycOcTree& _octree, const CPose& _pose)
{
    CCycOcTree transformed_octree(_octree.getResolution());
    
    for (auto it = _octree.begin_leafs(), end = _octree.end_leafs(); it != end; ++it)
    {
        const Eigen::Vector4f before_transform(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z(), 1.f);
        const Eigen::Vector4f after_transform = _pose.transform() * before_transform;

        auto* node = transformed_octree.updateNode(after_transform(0), after_transform(1), after_transform(2), true);
        node->setColor(it->getColor());
        node->setValue(it->getValue());
        node->setObjectClass(it->getObjectClass());
    }

    _octree.swapContent(transformed_octree);
}

void COcTreeUtils::octree2gridmap(const CCycOcTree& _octree, Eigen::MatrixXi& _gridmap, const Eigen::Vector2f _size)
{
    // TODO - OCTREE_RESOLUTION
    const float scaling_factor = 1.F / (float)_octree.getResolution();
    const auto rows = (CyC_UINT)(_size.y() * scaling_factor);
    const auto cols = (CyC_UINT)(_size.x() * scaling_factor);

    // Occupancy grid map structure
    _gridmap = Eigen::MatrixXi::Constant(rows, cols, -1);

    for (CCycOcTree::leaf_iterator it = _octree.begin_leafs(), end = _octree.end_leafs(); it != end; ++it)
    {
        const CyC_INT eigen_row_index = rows - (CyC_INT)(((it.getCoordinate().x() + _size.y() / 2.F)) * scaling_factor);
        const CyC_INT eigen_col_index = cols - (CyC_INT)(((it.getCoordinate().y() + _size.x() / 2.F)) * scaling_factor);

        if ((eigen_row_index > 0) && (eigen_col_index > 0) &&
            (eigen_row_index < (CyC_INT)rows) && (eigen_col_index < (CyC_INT)cols))
        {
            _gridmap(eigen_row_index, eigen_col_index) = it->getObjectClass();
        }
    }
}

void COcTreeUtils::octree2image(const CCycOcTree& _octree, const CBaseSensorModel* _sensor_model, cv::Mat& _dst)
{
    for (CCycOcTree::leaf_iterator it = _octree.begin_leafs(), end = _octree.end_leafs(); it != end; ++it)
    {
        CyC_INT cls = static_cast<CyC_INT>(it->getValue());
        cv::Scalar color = CObjectClasses::getColor(it->getObjectClass());

        Eigen::Vector4f pt3d_W(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z(), 1.f);
        Eigen::Vector4f pt3d_C = _sensor_model->extrinsics().inverse().transform() * pt3d_W;
        //pt3d_C = _sensor_model->extrinsics_inv().transform() * pt3d_C;

        const CPinholeCameraSensorModel* model = static_cast<const CPinholeCameraSensorModel*>(_sensor_model);
        CycPoint pt3d_reproj = CProjectiveGeometry::project(model, CPose(), pt3d_C);
        cv::Point2f cv_pt3d_reproj(pt3d_reproj.pt2d.x(), pt3d_reproj.pt2d.y());
        cv::circle(_dst, cv_pt3d_reproj, 1, color::blue, 3);
        
        // Calculate cube points
        /*const float cube_size = (float)it.getSize();
        const float cube_size_2 = cube_size * 0.5f;
        const float x = pt3d_C.x() - cube_size_2;
        const float y = pt3d_C.y() - cube_size_2;
        const float z = pt3d_C.z() - cube_size_2;
        const float width = cube_size;
        const float height = cube_size;
        const float depth = cube_size;

        std::vector<Eigen::Vector4f> pts_cube;
        pts_cube.push_back(Eigen::Vector4f{ x, y, z, 1.f });
        pts_cube.push_back(Eigen::Vector4f{ x + width, y, z , 1.f });
        pts_cube.push_back(Eigen::Vector4f{ x, y, z + depth, 1.f });
        pts_cube.push_back(Eigen::Vector4f{ x + width, y, z + depth, 1.f });
        pts_cube.push_back(Eigen::Vector4f{ x, y + height, z, 1.f });
        pts_cube.push_back(Eigen::Vector4f{ x + width, y + height, z, 1.f });
        pts_cube.push_back(Eigen::Vector4f{ x, y + height, z + depth, 1.f });
        pts_cube.push_back(Eigen::Vector4f{ x + width, y + height, z + depth, 1.f });

        std::vector<Eigen::Vector2f> pts_cube_proj;
        for (const auto& pt3d : pts_cube)
            pts_cube_proj.push_back(_sensor_model->world2sensor(pt3d));

        if (z > 0.f)
        {
            cv::line(_dst, cv::Point2f{ pts_cube_proj[0].x(), pts_cube_proj[0].y() }, cv::Point2f{ pts_cube_proj[1].x(), pts_cube_proj[1].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[1].x(), pts_cube_proj[1].y() }, cv::Point2f{ pts_cube_proj[3].x(), pts_cube_proj[3].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[3].x(), pts_cube_proj[3].y() }, cv::Point2f{ pts_cube_proj[2].x(), pts_cube_proj[2].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[2].x(), pts_cube_proj[2].y() }, cv::Point2f{ pts_cube_proj[0].x(), pts_cube_proj[0].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[4].x(), pts_cube_proj[4].y() }, cv::Point2f{ pts_cube_proj[5].x(), pts_cube_proj[5].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[5].x(), pts_cube_proj[5].y() }, cv::Point2f{ pts_cube_proj[7].x(), pts_cube_proj[7].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[7].x(), pts_cube_proj[7].y() }, cv::Point2f{ pts_cube_proj[6].x(), pts_cube_proj[6].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[6].x(), pts_cube_proj[6].y() }, cv::Point2f{ pts_cube_proj[4].x(), pts_cube_proj[4].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[0].x(), pts_cube_proj[0].y() }, cv::Point2f{ pts_cube_proj[4].x(), pts_cube_proj[4].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[1].x(), pts_cube_proj[1].y() }, cv::Point2f{ pts_cube_proj[5].x(), pts_cube_proj[5].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[2].x(), pts_cube_proj[2].y() }, cv::Point2f{ pts_cube_proj[6].x(), pts_cube_proj[6].y() }, color, 1);
            cv::line(_dst, cv::Point2f{ pts_cube_proj[3].x(), pts_cube_proj[3].y() }, cv::Point2f{ pts_cube_proj[7].x(), pts_cube_proj[7].y() }, color, 1);
        }*/
    }
}
