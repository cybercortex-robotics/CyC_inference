// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CImageDisplayUtils.h"
#include <iostream>

std::unique_ptr<CCycCache>      CDrawer::m_pSlamDispCache = nullptr;
std::vector<Eigen::Vector3f>    CDrawer::m_SlamTrajectory;

CImageDisplayUtils::CImageDisplayUtils()
{}

CImageDisplayUtils::~CImageDisplayUtils()
{}

void CImageDisplayUtils::drawPoseCamView(const cv::Mat& rmat,
    const cv::Mat& tvec,
    const cv::Mat& K,
    const cv::Mat& D,
    cv::Mat& dst,
    const float& length,
    const std::string& label)
{
    cv::Mat rvec;
    cv::Rodrigues(rmat, rvec);

    // Coordinates of the pose origin
    cv::Mat rvecOrigin = (cv::Mat_<float>(3, 1) << 0., 0., 0.);
    cv::Mat tvecOrigin = (cv::Mat_<float>(3, 1) << 0., 0., 0.);
    std::vector<cv::Point3f> ptOrigin3D;
    std::vector<cv::Point2f> ptOrigin2D;
    ptOrigin3D.push_back(cv::Point3f(tvec.at<float>(0), tvec.at<float>(1), tvec.at<float>(2)));
    cv::projectPoints(ptOrigin3D, rvecOrigin, tvecOrigin, K, D, ptOrigin2D);
    
    // Check if the pose is in the image plane
    if (ptOrigin2D[0].x >= 0 && ptOrigin2D[0].x < dst.cols && ptOrigin2D[0].y >= 0 && ptOrigin2D[0].y < dst.rows)
    {
        // Create the 3D points of the pose axes, which will be projected onto the image plane
        std::vector<cv::Point3f> pts3D;
        pts3D.push_back(cv::Point3f(0.1f, 0.f, 0.f)); // X-axis
        pts3D.push_back(cv::Point3f(0.f, 0.1f, 0.f)); // Y-axis
        pts3D.push_back(cv::Point3f(0.f, 0.f, 0.1f)); // Z-axis pointing towards the camera
        
        // Project the 3D axes points to 2D
        std::vector<cv::Point2f> pts2D;
        cv::projectPoints(pts3D, rvec, tvec, K, D, pts2D);

        // Draw the projected axes with respect to the pose origin
        if (tvec.at<float>(2) >= 0.)
        {
            cv::line(dst, ptOrigin2D[0], pts2D[0], CV_RGB(0, 255, 0), 2);    // X-axis in green
            cv::line(dst, ptOrigin2D[0], pts2D[1], CV_RGB(0, 162, 232), 2);  // Y-axis in blue
            cv::line(dst, ptOrigin2D[0], pts2D[2], CV_RGB(255, 0, 0), 2);    // Z-axis in red
        }
        else
        {
            cv::line(dst, ptOrigin2D[0], pts2D[2], CV_RGB(255, 0, 0), 2);    // Z-axis in red
            cv::line(dst, ptOrigin2D[0], pts2D[0], CV_RGB(0, 255, 0), 2);    // X-axis in green
            cv::line(dst, ptOrigin2D[0], pts2D[1], CV_RGB(0, 162, 232), 2);  // Y-axis in blue
        }
    }
    
    if (label.size() > 0)
        cv::putText(dst, label, cv::Point(ptOrigin2D[0].x - 18, ptOrigin2D[0].y + 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 0, 255), 1);
}

void CImageDisplayUtils::drawPoseCamView(const Eigen::Affine3f& T,
    const cv::Mat& K,
    const cv::Mat& D,
    cv::Mat& dst,
    const float& length,
    const std::string& label)
{
    cv::Mat rmatCamera, tvecCamera;

    cv::eigen2cv(T.rotation(), rmatCamera);
    cv::eigen2cv(Eigen::Vector3f(T.translation()), tvecCamera);

    drawPoseCamView(rmatCamera, tvecCamera, K, D, dst, length, label);
}

void CImageDisplayUtils::drawPoseCamView(const CPose& pose,
    const cv::Mat& K,
    const cv::Mat& D,
    cv::Mat& dst,
    const float& length,
    const std::string& label)
{
    cv::Mat rmatCamera, tvecCamera;
    
    cv::eigen2cv(pose.rotation_3x3(), rmatCamera);
    cv::eigen2cv(pose.translation_3x1(), tvecCamera);

    drawPoseCamView(rmatCamera, tvecCamera, K, D, dst, length, label);

    //cv::drawFrameAxes(dst, K, D, rmatCamera, tvecCamera, 0.1);
}

void CImageDisplayUtils::drawObjects(cv::Mat& dst, const CycRois2D& objects, std::unordered_map<CyC_INT, std::string>& obj_classes_map, CycDatablockKey key)
{
	double alpha = 0.3;
    float scaling = 2.f;
    cv::resize(dst, dst, cv::Size(dst.cols * scaling, dst.rows * scaling));

	for (CyC_INT i = 0; i < objects.size(); ++i)
	{
        CycRoi2D obj = objects[i];
        obj.origin.x() *= dst.cols; //scaling;
        obj.origin.y() *= dst.rows; //scaling;
        obj.width *= dst.cols;      //scaling;
        obj.height *= dst.rows;     //scaling;

        if ((key.nCoreID == -1 || key.nFilterID == -1) || (obj.key == key))
        {
            if (obj.origin.x() <= 0.f || obj.origin.y() <= 0.f ||
                obj.origin.x() + obj.width > dst.cols - 1 ||
                obj.origin.y() + obj.height > dst.rows - 1)
                continue;

            // Assign color based on class
            const auto& color = color::colormap[(obj.cls < sizeof(color::colormap)) ? obj.cls : 0];
            cv::Rect objRect(obj.origin.x(), obj.origin.y(), obj.width, obj.height);
            cv::Mat roi = dst(objRect);
            cv::Mat coloredRegion(roi.size(), CV_8UC3, color);
            cv::addWeighted(coloredRegion, alpha, roi, 1.0 - alpha, 0.0, roi);

            if (obj.origin.y() - 20 > 0)
            {
                //const std::string sLabel = (obj.cls <= labels.size()) ? labels[obj.cls - 1] : std::to_string(obj.cls);
                const std::string sLabel = obj_classes_map[obj.cls];
                const std::string sID = std::to_string(obj.id);

                const cv::Size textSize = cv::getTextSize(sLabel, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, nullptr);
                cv::Rect objRectLabel(obj.origin.x(), obj.origin.y() - 20, obj.width / 2, 30);
                cv::Rect objRectID(obj.origin.x() + obj.width / 2, obj.origin.y() - 20, obj.width / 2, 30);

                if ((objRectLabel.x >= 0) && (objRectLabel.y >= 0) &&
                    ((objRectLabel.x + objRectLabel.width) < dst.cols) &&
                    ((objRectLabel.y + objRectLabel.height) < dst.rows))
                {
                    roi = dst(objRectLabel);
                    cv::Mat colorLabel(roi.size(), CV_8UC3, color);
                    cv::addWeighted(colorLabel, alpha, roi, 0.1, 0.0, roi);

                    roi = dst(objRectID);
                    cv::Mat colorID(roi.size(), CV_8UC3, color);
                    cv::addWeighted(colorID, alpha, roi, 0.1, 0.0, roi);

                    cv::putText(dst, sLabel, cv::Point(obj.origin.x(), obj.origin.y()), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, color::white, 1);
                    //cv::putText(dst, sID, cv::Point(obj.origin.x() + obj.width / 2, obj.origin.y()), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, color::white, 1);
                    cv::putText(dst, sID, cv::Point(obj.origin.x() + 5, obj.origin.y() + 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, color::white, 1);
                }
            }

            cv::Rect rect = objRect;
            rect.y -= 20;
            rect.height += 20;
            cv::rectangle(dst, rect, color);
        }
	}

    cv::resize(dst, dst, cv::Size(dst.cols / scaling, dst.rows / scaling));
}

void CImageDisplayUtils::drawPanoSemSeg(cv::Mat& dst, const CycImage_& sem_obj)
{
    if (sem_obj.empty1() || dst.empty())
        return;

    cv::Mat tmp_img{ (int)sem_obj.nRows, (int)sem_obj.nCols, (int)sem_obj.nType1, sem_obj.pData1 };
    cv::resize(tmp_img, tmp_img, cv::Size(dst.cols, dst.rows));
    cv::addWeighted(dst, 0.6, tmp_img, 0.8, 0.0, dst);
    
    //tmp_img.copyTo(dst);
}

void CImageDisplayUtils::drawSemanticSegmentation(cv::Mat& dst, const CycImage_& sem_obj, std::vector<CyC_UINT> classes)
{
    if (sem_obj.empty1())
        return;
    
    cv::Mat tmp_img{ (int)sem_obj.nRows, (int)sem_obj.nCols, (int)sem_obj.nType1, sem_obj.pData1 };

    if (tmp_img.channels() > 1)
        cv::cvtColor(tmp_img, tmp_img, cv::COLOR_BGR2GRAY);

    cv::Mat seg_image = cv::Mat::zeros(tmp_img.rows, tmp_img.cols, CV_8UC3);
    cv::Mat dst_resized;
    cv::resize(dst, dst_resized, cv::Size(seg_image.cols, seg_image.rows));
    
    for (CyC_INT i = 0; i < tmp_img.rows; ++i)
    {
        for (CyC_INT j = 0; j < tmp_img.cols; ++j)
        {
            unsigned char pixel_cls = tmp_img.at<unsigned char>(i, j);
            cv::Scalar c = color::colormap[pixel_cls];
            seg_image.at<cv::Vec3b>(i, j) = {(unsigned char)c[0], (unsigned char)c[1], (unsigned char)c[2]};
        }
    }

    cv::resize(seg_image, seg_image, cv::Size(dst.cols, dst.rows));
    cv::addWeighted(seg_image, 0.75, dst, 0.85, 0.0, dst);
}

void CImageDisplayUtils::drawLanes(cv::Mat& dst, const CycLanesModel& lanes_model)
{
    int colors[5][3] = { {0, 255, 0},{255, 255, 0},{0, 255, 255},{255, 0, 0},{0, 0, 255} };
    for (auto i = 0; i < lanes_model.size(); ++i)
    {
        bool drew_once = false;

        Eigen::Vector2f pt2D_prev(-1.f, -1.f);
        for (float x = 0; x <= dst.cols; x += 3.f)
        {
            float y = CPolynomialFitting::polyeval(lanes_model[i].model, x);

            if (y > dst.rows / 1.7f)
            {
                Eigen::Vector2f pt2D{ x, y };

                if (pt2D_prev.x() != -1.f && pt2D_prev.y() != -1.f)
                    cv::line(
                        dst,
                        cv::Point{ (CyC_INT)(pt2D_prev[0]),
                        (CyC_INT)(pt2D_prev[1]) },
                        cv::Point{ (CyC_INT)(pt2D[0]),
                        (CyC_INT)(pt2D[1]) },
                        CV_RGB(colors[i % 5][0], colors[i % 5][1], colors[i % 5][2]),
                        2);

                pt2D_prev = pt2D;
                drew_once = true;
            }
            else if (drew_once)
                break;
        }
    }
}

void CImageDisplayUtils::drawLanes(cv::Mat& dst, const CycLanesModel& lanes_model, const CPinholeCameraSensorModel& camera_model)
{
    Eigen::Matrix4f M_veh2cam = camera_model.pose().transform().inverse();
    //M_veh2cam << 0.f, -1.f, -0.f, 0.0159f,
    //    0.f, 0.f, -1.f, 1.5f,
    //    1.f, 0.f, 0.f, 1.7f,
    //    0.f, 0.f, 0.f, 1.f;

    for (auto i = 0; i < lanes_model.size(); ++i)
    {
        Eigen::Vector2f pt2D_prev(-1.f, -1.f);
        for (float X = 0.f; X <= 100.f; X += 2.f)
        {
            float Y = CPolynomialFitting::polyeval(lanes_model[i].model, X);
            float Z = 0.f;

            // Convert to from vehicle to camera coordinates
            Eigen::Vector4f pt3dCam = M_veh2cam * Eigen::Vector4f{ X, Y, Z, 1 };

            // Project points to image
            Eigen::Vector2f pt2D = camera_model.world2sensor(pt3dCam);

            // Draw lane
            if (pt3dCam.z() > 0.f)
            {
                if (pt2D_prev.x() != -1.f && pt2D_prev.y() != -1.f)
                    cv::line(dst, cv::Point{ (CyC_INT)(pt2D_prev[0]), (CyC_INT)(pt2D_prev[1]) }, cv::Point{ (CyC_INT)(pt2D[0]), (CyC_INT)(pt2D[1]) }, CV_RGB(0, 255, 0), 2);
                
                pt2D_prev = pt2D;
            }
        }
    }

}

void CImageDisplayUtils::drawPoints(cv::Mat& dst, const CycPoints& pts, CyC_INT& last_track_id)
{
    char str[128];
    static std::unordered_map<std::size_t, cv::Scalar> track_colors;

    float scaling = 2.f;
    cv::resize(dst, dst, cv::Size(dst.cols * scaling, dst.rows * scaling));

    // If the unordered map has been initialized with an empty key
    if (pts.empty())
        return;
    
    for (CycPoint pt : pts)
    {
        const int& track_id = pt.id;
        cv::Scalar track_color(0, 255, 0);

        //if (last_track_id < track_id)
        //{
        //    // generate random color, but don't use it yet
        //    CyC_INT channel_b = rand() % 255;
        //    CyC_INT channel_g = rand() % 255;
        //    CyC_INT channel_r = rand() % 255;
        //    track_colors[(std::size_t)track_id] = CV_RGB(channel_r, channel_g, channel_b);
        //    track_color = track_colors[track_id];
        //}
        //else
        //{
        //    // tracked feature: use old color
        //    track_color = track_colors[track_id];
        //}

        cv::Point cv_pt((CyC_INT)pt.pt2d.x() * scaling, (CyC_INT)pt.pt2d.y() * scaling);

        cv::Rect pt_rect{ cv_pt.x - 3, cv_pt.y - 3, 6, 6 };
        cv::rectangle(dst, pt_rect, track_color, 1);
        cv::circle(dst,
            cv_pt,
            1,
            track_color,
            1, // thickness
            8 // line type
        );

        snprintf(str, sizeof(str) - 1, "%.2f", pt.score);
        cv::putText(dst, str, cv::Point(cv_pt.x + 6, cv_pt.y), cv::FONT_HERSHEY_PLAIN, 1, track_color, 1);
    }

    // update the highest track id
    if (pts.back().id > last_track_id)
    {
        last_track_id = pts.back().id;
    }

    cv::resize(dst, dst, cv::Size(dst.cols / scaling, dst.rows / scaling));
}

void CImageDisplayUtils::drawPoints(cv::Mat& dst, const CycPoints& _pts, const cv::Scalar& _color)
{
    for (size_t k = 0; k < _pts.size(); ++k)
    {
        // White points in the left image
        cv::Point cv_pt2d_1((CyC_INT)(_pts[k].pt2d.x() * dst.cols), (CyC_INT)(_pts[k].pt2d.y() * dst.rows));
        //cv::circle(dst, cv_pt2d_1, 1, _color, 2);
        cv::Rect pt_rect{ cv_pt2d_1.x - 6, cv_pt2d_1.y - 6, 12, 12 };
        cv::rectangle(dst, pt_rect, _color, 2);
    }
}

void CImageDisplayUtils::drawTrackedPoints(cv::Mat& dst, const CycPoints& pts_curr, const CycPoints& pts_prev)
{
    static std::unordered_map<std::size_t, cv::Scalar> track_colors;

    // If the unordered map has been initialized with an empty key
    if (pts_curr.empty())
        return;

    for (CycPoint pt_curr : pts_curr)
    {
        const int& track_id = pt_curr.id;
        cv::Scalar track_color(255, 255, 255);

        // Find in current point in previous points
        bool bFound = false;
        for (CycPoint pt_prev : pts_prev)
            if (pt_curr.id == pt_prev.id)
                bFound = true;

        if (!bFound)
        {
            // generate random color, but don't use it yet
            int channel_b = rand() % 255;
            int channel_g = rand() % 255;
            int channel_r = rand() % 255;
            track_colors[(std::size_t)track_id] = CV_RGB(channel_r, channel_g, channel_b);
            track_color = track_colors[track_id];
        }
        else
        {
            // tracked feature: use old color
            track_color = track_colors[track_id];
        }

        cv::Rect pt_rect{ (int)pt_curr.pt2d.x() - 4, (int)pt_curr.pt2d.y() - 4, 8, 8 };
        cv::rectangle(dst, pt_rect, track_color);
        cv::circle(dst,
            cv::Point((int)pt_curr.pt2d.x(), (int)pt_curr.pt2d.y()),
            1,
            track_color,
            1, // thickness
            8 // line type
        );
    }
}

void CImageDisplayUtils::drawVoxels(cv::Mat& dst, const CycVoxels& voxels, const CPinholeCameraSensorModel& camera_model, const cv::Scalar& _color)
{
    for (const CycVoxel& voxel : voxels)
    {
        Eigen::Vector2f pt2D = camera_model.world2sensor(voxel.pt3d);
        
        cv::Rect rect = cv::Rect((CyC_INT)pt2D.x() - 4, (CyC_INT)pt2D.y() - 4, 8, 8);
        cv::rectangle(dst, rect, _color, 1);
    }
} 

void CImageDisplayUtils::drawVehicleControlInput(cv::Mat& dst, const CycControlInput& control_input, const CPinholeCameraSensorModel& camera_model)
{
    Eigen::Vector2f pt2d_prev{ -1.f, -1.f };
    Eigen::Matrix4f M_w2cam = camera_model.pose().transform().inverse();

    for (const auto& pt3d_w : control_input.ref_pts.ref)
    {
        Eigen::Vector4f pt3d_cam = M_w2cam * Eigen::Vector4f{ pt3d_w.x(), pt3d_w.y(), 0.F, 1.F };
        Eigen::Vector2f pt2d = camera_model.world2sensor(pt3d_cam);

        // Draw reference trajectory
        if (pt3d_cam.z() > 0.f)
        {
            if (pt2d_prev.x() != -1.f && pt2d_prev.y() != -1.f)
                cv::line(dst, cv::Point{ (CyC_INT)(pt2d_prev[0]), (CyC_INT)(pt2d_prev[1]) }, cv::Point{ (CyC_INT)(pt2d[0]), (CyC_INT)(pt2d[1]) }, color::reference_path, 2);

            pt2d_prev = pt2d;
        }
    }
}

void CImageDisplayUtils::drawEpipolarLines(const cv::Mat F,
    const cv::Mat& img1,
    const cv::Mat& img2,
    const std::vector<cv::Point2f>& points1,
    const std::vector<cv::Point2f>& points2,
    cv::Mat& dst,
    const bool bDrawLeftEpilines,
    const bool bDrawRightEpilines,
    const float inlierDistance)
{
    CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
    
    dst = cv::Mat (img1.rows, img1.cols * 2, CV_8UC3);
    cv::Rect rect1(0, 0, img1.cols, img1.rows);
    cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
    
    /*
     * Allow color drawing
     */
    if (img1.type() == CV_8U)
    {
        cv::cvtColor(img1, dst(rect1), cv::COLOR_GRAY2BGR);
        cv::cvtColor(img2, dst(rect2), cv::COLOR_GRAY2BGR);
    }
    else
    {
        img1.copyTo(dst(rect1));
        img2.copyTo(dst(rect2));
    }
    std::vector<cv::Vec3f> epilines1, epilines2;
    cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
    cv::computeCorrespondEpilines(points2, 2, F, epilines2);

    CV_Assert(points1.size() == points2.size() &&
        points2.size() == epilines1.size() &&
        epilines1.size() == epilines2.size());

    cv::RNG rng(0);
    for (size_t i = 0; i < points1.size(); i++)
    {
        if (inlierDistance > 0)
        {
            if (distancePointLine(points1[i], epilines2[i]) > inlierDistance ||
                distancePointLine(points2[i], epilines1[i]) > inlierDistance)
            {
                //The point match is no inlier
                continue;
            }
        }
        /*
         * Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
         */
        cv::Scalar color(rng(256), rng(256), rng(256));

        if (bDrawLeftEpilines)
        {
            cv::line(dst(rect2),
                cv::Point(0, -epilines1[i][2] / epilines1[i][1]),
                cv::Point(img1.cols, -(epilines1[i][2] + epilines1[i][0] * img1.cols) / epilines1[i][1]),
                color,
                2);
            cv::circle(dst(rect1), points1[i], 4, color, -1);
        }
        
        if(bDrawRightEpilines)
        {
            cv::line(dst(rect1),
                cv::Point(0, -epilines2[i][2] / epilines2[i][1]),
                cv::Point(img2.cols, -(epilines2[i][2] + epilines2[i][0] * img2.cols) / epilines2[i][1]),
                color,
                2);
            cv::circle(dst(rect2), points2[i], 4, color, -1);
        }
    }
}

void CImageDisplayUtils::drawLocalEgoVehicleEnvironment(const CycState& _vehicle_state, cv::Mat* _disp_ptr)
{
    cv::Mat tmp;
    cv::Mat& _disp = (_disp_ptr == nullptr) ? tmp : *_disp_ptr;

    cv::circle(_disp, cv::Point(50, 50), 2, CV_RGB(255, 0, 0));
}

float CImageDisplayUtils::distancePointLine(const cv::Point2f point, const cv::Vec3f& line)
{
    //Line is given as a*x + b*y + c = 0
    return fabsf(line(0)*point.x + line(1)*point.y + line(2))
        / std::sqrt(line(0)*line(0) + line(1)*line(1));
}

cv::Mat CImageDisplayUtils::concatImages(std::vector<cv::Mat> & images, int cols, int min_gap_size)
{
    // let's first find out the maximum dimensions
    int max_width = 0;
    int max_height = 0;
    for (int i = 0; i < images.size(); i++) {
        // check if type is correct 
        // you could actually remove that check and convert the image 
        // in question to a specific type
        if (i > 0 && images[i].type() != images[i - 1].type()) {
            std::cerr << "WARNING:createOne failed, different types of images";
            return cv::Mat();
        }
        max_height = std::max(max_height, images[i].rows);
        max_width = std::max(max_width, images[i].cols);
    }
    // number of images in y direction
    int rows = std::ceil(images.size() / cols);

    // create our result-matrix
    cv::Mat result = cv::Mat::zeros(rows*max_height + (rows - 1)*min_gap_size,
        cols*max_width + (cols - 1)*min_gap_size, images[0].type());
    size_t i = 0;
    int current_height = 0;
    int current_width = 0;
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            if (i >= images.size()) // shouldn't happen, but let's be safe
                return result;
            // get the ROI in our result-image
            cv::Mat to(result,
                cv::Range(current_height, current_height + images[i].rows),
                cv::Range(current_width, current_width + images[i].cols));
            // copy the current image to the ROI
            images[i++].copyTo(to);
            current_width += max_width + min_gap_size;
        }
        // next line - reset width and update height
        current_width = 0;
        current_height += max_height + min_gap_size;
    }
    return result;
}

cv::Mat CImageDisplayUtils::concatTemporalImagesHorizontal(std::vector<std::vector<cv::Mat>>& images)
{
    cv::Mat cvMergedTemporalImages;

    for (auto i = 0; i < images.size(); ++i)
    {
        if (images[i].size() > 0)
        {
            cv::Mat cvTemporalSeq = concatImages(images[i], (CyC_INT)images[i].size(), 0);

            if (i == 0)
            {
                // Print the temporal information
                for (auto k = 0; k < images[i].size(); ++k)
                {
                    char str[128];
                    if (k == images[i].size() - 1)
                        snprintf(str, sizeof(str) - 1, "t");
                    else
                        snprintf(str, sizeof(str) - 1, "t - %zd", images[i].size() - k - 1);
                    cv::putText(cvTemporalSeq, str, cv::Point(15 + k * images[i][0].cols, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, color::black, 1);
                }
                cvMergedTemporalImages = cvTemporalSeq;
            }
            else
            {
                if (cvMergedTemporalImages.cols > cvTemporalSeq.cols)
                {
                    // fill space in cvMergedGrids
                    cv::Mat cvFill(cvTemporalSeq.rows, cvMergedTemporalImages.cols - cvTemporalSeq.cols, cvTemporalSeq.type(), color::black);
                    cv::hconcat(cvTemporalSeq, cvFill, cvTemporalSeq);
                }
                else if (cvTemporalSeq.cols > cvMergedTemporalImages.cols)
                {
                    // fill space in cvMergedCameraImages
                    cv::Mat cvFill(cvMergedTemporalImages.rows, cvTemporalSeq.cols - cvMergedTemporalImages.cols, cvMergedTemporalImages.type(), color::black);
                    cv::hconcat(cvMergedTemporalImages, cvFill, cvMergedTemporalImages);
                }

                cv::vconcat(cvMergedTemporalImages, cvTemporalSeq, cvMergedTemporalImages);
            }
        }
    }

    return cvMergedTemporalImages;
}

cv::Mat CImageDisplayUtils::concatTemporalImagesVertical(std::vector<std::vector<cv::Mat>>& images)
{
    char str[128];
    cv::Mat cvMergedTemporalImages;
    std::vector<cv::Mat> images_streams;

    for (auto i = 0; i < images.size(); ++i)
    {
        cv::Mat cvTemporalSeq;

        for (auto j = images[i].size(); j > 0; --j)
        {
            cv::Mat img = images[i][j - 1].clone();

            if (j == images[i].size())
            {
                if (i == 0)
                {
                    snprintf(str, sizeof(str) - 1, "t");
                    cv::putText(img, str, cv::Point(15, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, color::black, 1);
                }
                cvTemporalSeq = img;
            }
            else
            {
                if (i == 0)
                {
                    snprintf(str, sizeof(str) - 1, "t - %zi", images[i].size() - j);
                    cv::putText(img, str, cv::Point(15, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, color::black, 1);
                }
                cv::vconcat(cvTemporalSeq, img, cvTemporalSeq);
            }
        }

        images_streams.push_back(cvTemporalSeq);

        if (i == 0)
            cvMergedTemporalImages = cvTemporalSeq;
        else
            if (!cvTemporalSeq.empty() && !cvMergedTemporalImages.empty() &&
                cvTemporalSeq.cols == cvMergedTemporalImages.cols &&
                cvTemporalSeq.rows == cvMergedTemporalImages.rows)
                cv::hconcat(cvMergedTemporalImages, cvTemporalSeq, cvMergedTemporalImages);
    }

    return cvMergedTemporalImages;
}

cv::Point2f CImageDisplayUtils::rot(
    const cv::Point2f& pt,
    float theta)
{
    cv::Point2f ret;

    ret.x = cos(theta) * pt.x - sin(theta) * pt.y;
    ret.y = sin(theta) * pt.x + cos(theta) * pt.y;

    return ret;
}

cv::Point2f CImageDisplayUtils::rot_t(
    const cv::Point2f& pt,
    const cv::Point2f& orig,
    float theta)
{
    cv::Point2f ret = rot(pt, theta);

    // translate
    ret.x += orig.x;
    ret.y += orig.y;

    return ret;
}

cv::Mat CImageDisplayUtils::ultrasonicsToImage(const CycState& vehicleState, const CycUltrasonics& uss, CyC_UINT num_us_front, CyC_UINT num_us_rear)
{
    const auto rows = 250;
    const auto cols = 250;
    const cv::Point2f vehiclePosition{ rows / 2.f, cols / 2.f };
    const auto _vehicle_heading = vehicleState.x_hat[3];

    cv::Mat usMat = cv::Mat::zeros(rows, cols, CV_8UC3);

    // The ultrasonic maximum range is at 80% of the minimum of (rows/2 and cols/2)
    const auto maxUsVal = uss[0].max_range;
    const auto scaleFactor = 0.8f * std::min(rows / 2, cols / 2) / maxUsVal;
    std::vector<cv::Point> poly_points;
    CyC_UINT index = 0;
    cv::Point start_point_front = cv::Point(0.f, 0.f);
    cv::Point end_point_front = cv::Point(0.f, 0.f);
    cv::Point start_point_rear = cv::Point(0.f, 0.f);
    cv::Point end_point_rear = cv::Point(0.f, 0.f);
    for (auto us : uss)
    {
        const auto fillX = scaleFactor * (us.pose.translation_3x1()(0) + us.range * cosf(-us.pose.rotation_euler()(2)));
        const auto fillY = scaleFactor * (us.pose.translation_3x1()(1) + us.range * sinf(-us.pose.rotation_euler()(2)));

        //US start point
        const auto X1 = scaleFactor * us.pose.translation_3x1()(0);
        const auto Y1 = scaleFactor * us.pose.translation_3x1()(1);

        //US end point
        const auto X2 = X1 + fillX;
        const auto Y2 = Y1 + fillY;

        // Rotate with -vehicle_heading
        const auto rot_s = rot(cv::Point2f{ X1, Y1 }, -_vehicle_heading);
        const auto rot_e = rot(cv::Point2f{ X2, Y2 }, -_vehicle_heading);


        // Translate back with vehiclePosition
        const auto us_start = cv::Point{ static_cast<CyC_INT>(rot_s.x + vehiclePosition.x),
            static_cast<CyC_INT>(rot_s.y + vehiclePosition.y) };

        const auto us_stop = cv::Point(static_cast<CyC_INT>(rot_e.x + vehiclePosition.x),
            static_cast<CyC_INT>(rot_e.y + vehiclePosition.y));

        if (index == 0 || index == num_us_front)
        {
            poly_points.push_back(us_start);
        }
        poly_points.push_back(us_stop);
        if (index == num_us_front - 1)
        {
            poly_points.push_back(us_start);
        }
        if (index == uss.size() - 1)
        {
            poly_points.push_back(us_start);
        }

        index++;
    }
    std::vector<std::vector<cv::Point>> f_poly_points;
    f_poly_points.push_back(poly_points);
    cv::fillPoly(usMat, f_poly_points, color::green);

    // Draw the vehicle
    cv::line(usMat,
        rot_t(cv::Point2f{ 0.F, -13.f }, vehiclePosition, -_vehicle_heading),
        rot_t(cv::Point2f{ 33.F, 0.F }, vehiclePosition, -_vehicle_heading),
        color::vehicle, 2);
    cv::line(usMat,
        rot_t(cv::Point2f{ 0.F,  13.f }, vehiclePosition, -_vehicle_heading),
        rot_t(cv::Point2f{ 33.F, 0.F }, vehiclePosition, -_vehicle_heading),
        color::vehicle, 2);
    cv::line(usMat,
        rot_t(cv::Point2f{ 0.F, -13.f }, vehiclePosition, -_vehicle_heading),
        rot_t(cv::Point2f{ 0.F,  13.f }, vehiclePosition, -_vehicle_heading),
        color::vehicle, 2);

    return usMat;
}


cv::Mat CImageDisplayUtils::concatTemporalUltrasonicRepresentation(const std::vector<cv::Mat>& usImages)
{
    const auto numCols = 1250;
    const auto numImagesPerRow = numCols / usImages[0].cols;
    const auto numRowsOfImages = usImages.size() / numImagesPerRow;
    const auto numRows = numRowsOfImages *  usImages[0].rows;
    const auto numImgCols = usImages[0].cols;
    const auto numImgRows = usImages[0].rows;

    cv::Mat result = cv::Mat::zeros(static_cast<int>(numRows), static_cast<int>(numCols), CV_8UC3);
    auto current_height = 0;
    auto current_width = 0;
    size_t i = 0;
    for (int y = 0; y < numRowsOfImages; y++) 
    {
        for (int x = 0; x < numImagesPerRow; x++) 
        {
            // get the ROI in our result-image
            cv::Mat to(result,
                cv::Range(current_height, current_height + usImages[i].rows),
                cv::Range(current_width, current_width + usImages[i].cols));
            // copy the current image to the ROI
            usImages[i++].copyTo(to);
            current_width += numImgCols;
        }
        current_width = 0;
        current_height += numImgRows;
    }
    return result;
}

void CImageDisplayUtils::printTimestamp(cv::Mat& _dst, const CyC_TIME_UNIT& _ts, const cv::Scalar& _color)
{
    CyC_INT nTextOffsetY = 17;
    std::string ts_as_string = CTimer::toString(_ts).c_str();
    cv::putText(_dst, ts_as_string.substr(0, ts_as_string.size() - 1), cv::Point(5, nTextOffsetY), cv::FONT_HERSHEY_PLAIN, 1.1, _color, 1);
}

//void CImageDisplayUtils::printDatastreamInfo(cv::Mat& _dst, const CyC_TIME_UNIT& _ts, CCycFilterBase* _filter)
//{
//    cv::Scalar TextColor = CV_RGB(100, 100, 255);
//    CyC_INT nTextOffsetY = 17;
//
//    if (_filter != nullptr)
//    {
//        char str[128];
//        snprintf(str, sizeof(str) - 1, "[%d, %d] %s", _filter->getFilterKey().nCoreID, _filter->getFilterKey().nFilterID, _filter->getFilterName());
//        cv::putText(_dst, str, cv::Point(5, nTextOffsetY), cv::FONT_HERSHEY_PLAIN, 1.1, TextColor, 1);
//    }
//
//    std::string ts_as_string = CTimer::toString(_ts).c_str();
//    cv::putText(_dst, ts_as_string.substr(0, ts_as_string.size() - 1), cv::Point(5, 2 * nTextOffsetY), cv::FONT_HERSHEY_PLAIN, 1.1, TextColor, 1);
//}

void CImageDisplayUtils::draw_correspondences(const cv::Mat& _img1, const cv::Mat& _img2,
    cv::Mat& _dst,
    const std::vector<Eigen::Vector2f>& _pts1,
    const std::vector<Eigen::Vector2f>& _pts2,
    const CyC_INT& _line_thickness)
{
    assert(_pts1.size() != _pts2.size());
    assert(_img1.channels() != _img2.channels());

    cv::Scalar m_color_curr_pts_2d = CV_RGB(92, 255, 132);
    double font_scale = _img1.rows / 200.;

    cv::hconcat(_img1, _img2, _dst);

    if (_dst.channels() < 3)
        cv::cvtColor(_dst, _dst, cv::COLOR_GRAY2RGB);

    for (size_t i = 0; i < _pts1.size(); ++i)
    {
        cv::Point2f cv_pt_1{ _pts1[i].x(), _pts1[i].y() };
        cv::Point2f cv_pt_2{ _pts2[i].x() + _img1.cols, _pts2[i].y() };
        cv::line(_dst, cv_pt_1, cv_pt_2, m_color_curr_pts_2d, _line_thickness);
    }

    cv::putText(_dst, "image 1 (previous)", cv::Point(5, _dst.rows - 25), cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 255, 255), 2);
    cv::putText(_dst, "image 2 (current)", cv::Point(5 + CyC_INT(_dst.cols / 2.f), _dst.rows - 25), cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 255, 255), 2);
}

void CImageDisplayUtils::draw_correspondences(const cv::Mat& _img1, const cv::Mat& _img2,
    cv::Mat& _dst,
    const CycPoints& _pts1, const CycPoints& _pts2, 
    const CyC_INT& _line_thickness,
    const std::vector<CyC_INT>& _matches)
{
    assert(_pts1.size() != _pts2.size());
    assert(_img1.channels() != _img2.channels());

    char str[128];
    double font_scale = _img1.rows / 200.;

    cv::hconcat(_img1, _img2, _dst);

    if (_dst.channels() < 3)
        cv::cvtColor(_dst, _dst, cv::COLOR_GRAY2RGB);

    for (size_t i = 0; i < _pts1.size(); ++i)
    {
        cv::Point2f cv_pt_1{ _pts1[i].pt2d.x(), _pts1[i].pt2d.y() };
        cv::Point2f cv_pt_2{ _pts2[i].pt2d.x() + _img1.cols, _pts2[i].pt2d.y() };

        bool bInMatches = false;
        for (const auto& idx : _matches)
            if (_pts1[i].id == idx)
                bInMatches = true;

        if (bInMatches)
            cv::line(_dst, cv_pt_1, cv_pt_2, color::green, _line_thickness);
        else
            cv::line(_dst, cv_pt_1, cv_pt_2, color::red, _line_thickness);

        snprintf(str, sizeof(str) - 1, "%d", _pts1[i].id);
        cv::putText(_dst, str, cv::Point2f(cv_pt_1.x - 20, cv_pt_1.y), cv::FONT_HERSHEY_PLAIN, font_scale / 1.5, color::blue, 1);
        cv::putText(_dst, str, cv_pt_2, cv::FONT_HERSHEY_PLAIN, font_scale / 1.5, color::blue, 1);
    }

    cv::putText(_dst, "image 1 (previous)", cv::Point(5, _dst.rows - 25), cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 255, 255), 2);
    cv::putText(_dst, "image 2 (current)", cv::Point(5 + CyC_INT(_dst.cols / 2.f), _dst.rows - 25), cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 255, 255), 2);
}

void CImageDisplayUtils::printT(cv::Mat& _img, std::string _text, const Eigen::Matrix4f _T, const CyC_INT _position, const cv::Scalar _color, const double _font_scale, const CyC_INT _thickness, const std::string _format)
{
    char str[128];

    if (_format.compare("euler") == 0)
    {
        Eigen::Vector3f euler = CPose::R2euler(_T.block(0, 0, 3, 3));
        
        snprintf(str, sizeof(str) - 1, "%s: R [%.2f; %.2f; %.2f]  t [%.2f; %.2f; %.2f]",
            _text.c_str(),
            euler.x() * RAD2DEG,
            euler.y() * RAD2DEG,
            euler.z() * RAD2DEG,
            _T.col(3).x(),
            _T.col(3).y(),
            _T.col(3).z());
    }
    else
    {
        CQuaternion quat = CQuaternion::rotmat2quaternion(_T.block(0, 0, 3, 3));
        
        snprintf(str, sizeof(str) - 1, "%s: R [%.2f; %.2f; %.2f; %.2f]  t [%.2f; %.2f; %.2f]",
            _text.c_str(),
            quat.x(),
            quat.y(),
            quat.z(),
            quat.w(),
            _T.col(3).x(),
            _T.col(3).y(),
            _T.col(3).z());
    }

    cv::putText(_img, str, cv::Point(5, _position), cv::FONT_HERSHEY_PLAIN, _font_scale, _color, _thickness);
}

void CImageDisplayUtils::drawLine(cv::Mat& _img, const Eigen::Vector3f& _line_coeffs, const cv::Scalar& _color)
{
    float x = 0;
    float y = (-_line_coeffs.z() - _line_coeffs.x() * x) / _line_coeffs.y();
    cv::Point2f cv_line_1(x, y);
    
    x = (float)(_img.cols);
    y = (-_line_coeffs.z() - _line_coeffs.x() * x) / _line_coeffs.y();
    cv::Point2f cv_line_2(x, y);
    
    cv::line(_img, cv_line_1, cv_line_2, _color);
}

void CImageDisplayUtils::drawLines(cv::Mat& _img, const std::vector<Eigen::Vector3f>& _lines_coeffs, const cv::Scalar& _color)
{
    for (size_t k = 0; k < _lines_coeffs.size(); ++k)
        CImageDisplayUtils::drawLine(_img, _lines_coeffs[k], _color);
}

cv::Mat CImageDisplayUtils::resizeKeepAspectRatio(const cv::Mat& _input_img, const cv::Size& _dst_size, const cv::Scalar& _bgcolor)
{
    cv::Mat output;

    float h1 = _dst_size.width * (_input_img.rows / (float)_input_img.cols);
    float w2 = _dst_size.height * (_input_img.cols / (float)_input_img.rows);

    if (h1 <= _dst_size.height)
        cv::resize(_input_img, output, cv::Size(_dst_size.width, h1));
    else
        cv::resize(_input_img, output, cv::Size(w2, _dst_size.height));

    CyC_INT top = (_dst_size.height - output.rows) / 2;
    CyC_INT down = (_dst_size.height - output.rows + 1) / 2;
    CyC_INT left = (_dst_size.width - output.cols) / 2;
    CyC_INT right = (_dst_size.width - output.cols + 1) / 2;

    cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, _bgcolor);

    return output;
}

void CImageDisplayUtils::draw_pose(cv::Mat& _out_dst, const float& _x, const float& _y, const float& _yaw, const float& _scale, const cv::Scalar& _color, const CyC_INT& _line_thickness)
{
    Eigen::Vector2f pt_origin(_x, -_y);
    pt_origin *= _scale;

    cv::Point2f cv_pt_origin(_out_dst.cols / 2.f + pt_origin.x(), _out_dst.rows / 2.f + pt_origin.y());
    float yaw_angle = /*-90.f * DEG2RAD*/ -_yaw;

    //std::cout << yaw_angle * RAD2DEG << std::endl;
    //if (yaw_angle > 90.f * DEG2RAD)
    //    yaw_angle = -yaw_angle;

    // Draw the vehicle
    cv::line(_out_dst,
        CImageDisplayUtils::rot_t(cv::Point2f{ 0.f, 0.f }, cv_pt_origin, yaw_angle),
        CImageDisplayUtils::rot_t(cv::Point2f{ 15.f, -17.f }, cv_pt_origin, yaw_angle),
        _color, _line_thickness);
    cv::line(_out_dst,
        CImageDisplayUtils::rot_t(cv::Point2f{ 0.f, 0.f }, cv_pt_origin, yaw_angle),
        CImageDisplayUtils::rot_t(cv::Point2f{ 15.f, 17.f }, cv_pt_origin, yaw_angle),
        _color, _line_thickness);
    cv::line(_out_dst,
        CImageDisplayUtils::rot_t(cv::Point2f{ 15.f, -17.f }, cv_pt_origin, yaw_angle),
        CImageDisplayUtils::rot_t(cv::Point2f{ 15.f, 17.f }, cv_pt_origin, yaw_angle),
        _color, _line_thickness);
}

void CImageDisplayUtils::show_slam_output(cv::Mat& _out_dst, const CPinholeCameraSensorModel* _cam_sensor_model, const CycSlam& _slam_data, const cv::Mat* _cam_view, const bool& _draw_curr_frame)
{
    cv::Mat disp_birds_view;
    float bew_scale = 25.0f; // 150.f; // 50.f; // 3.5f;
    cv::Size grid_size(_cam_sensor_model->width(), _cam_sensor_model->height());
    CImageDisplayUtils::draw_slam(disp_birds_view, grid_size, _slam_data, bew_scale, true, _draw_curr_frame, true);
    _out_dst = disp_birds_view;

    if (_cam_view != nullptr)
    {
        cv::Mat disp_cam_view;
        if (_cam_view->empty())
            disp_cam_view = cv::Mat::zeros(_cam_sensor_model->height(), _cam_sensor_model->width(), CV_8UC3);
        else
            disp_cam_view = *_cam_view;

        CImageDisplayUtils::draw_slam_frame(disp_cam_view, _cam_sensor_model, _slam_data);
        cv::resize(disp_birds_view, disp_birds_view, disp_cam_view.size());
        cv::hconcat(disp_cam_view, disp_birds_view, _out_dst);
    }
}

void CImageDisplayUtils::drawSlam(const cv::Mat& _img,
    cv::Mat& _out_dst,
    const CPinholeCameraSensorModel* _pSensorModel,
    const CPose& _abs_body_pose_W,
    const CPose& _abs_cam_pose_C,
    const CycPoints& prev_inliers_pts,
    const CycPoints& curr_inliers_pts,
    const CycVoxels& _voxels_prev_W,
    const CycVoxels& _voxels_curr_W,
    const float& _scale_factor,
    const std::vector<CyC_INT>& _scale_factor_samples_1,
    const std::vector<CyC_INT>& _scale_factor_samples_2,
    const bool _draw_epi_projections,
    const CCycCache* _preintegrated_imu_hist)
{
    //assert(prev_inliers_pts.size() != curr_inliers_pts.size());

    char str[128];
    CyC_INT nTextOffsetY = _img.rows / 20;
    double alpha = 0.85;
    double beta = 0.28;
    double gamma = 0.;
    double font_scale = 1.;
    CyC_INT font_thickness = 1;

    if (!_img.empty())
    {
        if (_img.channels() > 1)
        {
            cv::cvtColor(_img, _out_dst, cv::COLOR_RGB2GRAY);
            cv::cvtColor(_out_dst, _out_dst, cv::COLOR_GRAY2RGB);
        }
        else
        {
            _out_dst = _img;
        }
    }
    else
    {
        _out_dst = cv::Mat::zeros(cv::Size(_pSensorModel->width(), _pSensorModel->height()), CV_8UC3);
    }

    // Calculate the epipolar lines
    std::vector<Eigen::Vector3f> epi_lines_curr, epi_lines_prev;
    CProjectiveGeometry::epi_lines(_pSensorModel, CPose().transform(), _abs_cam_pose_C.transform(), prev_inliers_pts, curr_inliers_pts, epi_lines_prev, epi_lines_curr);

    // Draw keypoints
    CImageDisplayUtils::drawPoints(_out_dst, curr_inliers_pts, color::orthogonal_projection);

    // Calculate and draw the epipoles
    CycPoint epipole_curr = CProjectiveGeometry::epipole(_pSensorModel, _abs_cam_pose_C.transform(), CPose().transform());
    cv::Point2f cv_epipole_curr(epipole_curr.pt2d.x(), epipole_curr.pt2d.y());
    cv::circle(_out_dst, cv_epipole_curr, 4, CV_RGB(20, 255, 28), 2);

    // Draw epipolar lines
    CImageDisplayUtils::drawLines(_out_dst, epi_lines_curr, color::epi_lines);

    // Draw the orthogonal projections of each keypoint onto its epipolar line
    if (_draw_epi_projections)
    {
        for (size_t k = 0; k < prev_inliers_pts.size(); ++k)
        {
            Eigen::Vector2f orth_proj_curr = CGeometry::orthogonal_projection(epi_lines_curr[k], curr_inliers_pts[k].pt2d);
            cv::line(_out_dst, cv::Point(curr_inliers_pts[k].pt2d.x(), curr_inliers_pts[k].pt2d.y()), cv::Point(orth_proj_curr.x(), orth_proj_curr.y()), color::orthogonal_projection);
        }
    }

    // Reproject to 2D
    CycPoints pts_curr_reproj;
    //CycVoxels voxels_curr_C;
    //// TODO: previous voxels in camera coordinates should be calculated
    //CycVoxels voxels_prev_C = _voxels_prev_W;
    //Eigen::Matrix4f T_curr = _pSensorModel->extrinsics().transform() *_abs_body_pose_W.transform()* _pSensorModel->extrinsics().transform();
    //for (const auto& vx : _voxels_curr_W)
    //    voxels_curr_C.emplace_back(CycVoxel(T_curr * vx.pt3d, vx.id));

    //CProjectiveGeometry::project(_pSensorModel, _abs_cam_pose_C.transform(), _voxels_curr_W, pts_curr_reproj);

    // Compute projection matrices (required for computing the depth)
    Pmatrix P_cam_curr = CProjectiveGeometry::KT2P(_pSensorModel->K(), CProjectiveGeometry::invertT(_abs_cam_pose_C.transform()));

    // Reproject to 2D for visualization
    for (size_t k = 0; k < pts_curr_reproj.size(); ++k)
    {
        CycPoint pt = pts_curr_reproj[k];
        pt.pt2d.x() /= _pSensorModel->width();
        pt.pt2d.y() /= _pSensorModel->height();
        pt.pt2d.x() *= _out_dst.rows;
        pt.pt2d.y() *= _out_dst.cols;

        cv::Point2f cv_pt_curr(pt.pt2d.x(), pt.pt2d.y());
        cv::circle(_out_dst, cv_pt_curr, 10, color::blue, 2);

        //snprintf(str, sizeof(str) - 1, "%d", _voxels_curr_W[k].id);
        //cv::putText(_out_dst, str, cv::Point2f(pt.pt2d.x()+6, pt.pt2d.y()), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.65, color::blue, 1);
        
        /*cv::Point2f cv_pt_curr_reproj(pts_curr_reproj[k].pt2d.x(), pts_curr_reproj[k].pt2d.y());

        // Calculate depth in both projections
        float depth_curr = CTriangulation::getDepth(_voxels_curr_W[k], P_cam_curr);

        if (depth_curr > 0.f)
            cv::circle(_out_dst, cv_pt_curr_reproj, 7, color::reproj_pts_2d_pos_depth, 1);
        else
            cv::circle(_out_dst, cv_pt_curr_reproj, 7, color::reproj_pts_2d_neg_depth, 1);

        // Draw reprojection error
        cv::line(_out_dst, cv_pt_curr, cv_pt_curr_reproj, color::reprojection, 1);*/
    }

    // *** Draw correspondences ***
    cv::Mat img_corresp;
    cv::Mat img_corresp_prev = cv::Mat::zeros(_out_dst.rows, _out_dst.cols, CV_8UC3);
    cv::Mat img_corresp_curr = cv::Mat::zeros(_out_dst.rows, _out_dst.cols, CV_8UC3);
    CImageDisplayUtils::draw_correspondences(img_corresp_prev, img_corresp_curr, img_corresp, prev_inliers_pts, curr_inliers_pts, 2);
    
    // Draw the sampled voxels used for calculating the scale factor, if any
    //drawScaleFactor(img_corresp, _pSensorModel, _abs_cam_pose_C, voxels_prev_C, voxels_curr_C, _scale_factor_samples_1, _scale_factor_samples_2);

    // Stich correspondences image to the main display
    cv::resize(img_corresp, img_corresp, cv::Size(img_corresp.cols / 3.f, img_corresp.rows / 3.f));
    cv::Mat disp_rect_corresp = _out_dst(cv::Rect(0, _out_dst.rows - img_corresp.rows, img_corresp.cols, img_corresp.rows));
    cv::addWeighted(img_corresp, alpha, disp_rect_corresp, beta, gamma, disp_rect_corresp);

    // *** Draw preintegrated IMU data ***
    cv::Mat img_imu = cv::Mat::zeros(img_corresp.rows, _out_dst.cols - img_corresp.cols, CV_8UC3);
    drawPreintegratedImu(img_imu, _preintegrated_imu_hist);

    // Stich preintegrated IMU image to the main display
    //cv::resize(img_imu, img_imu, cv::Size(img_corresp.cols / 3.f, img_corresp.rows / 3.f));
    cv::Mat disp_rect_imu = _out_dst(cv::Rect(img_corresp.cols, _out_dst.rows - img_imu.rows, img_imu.cols, img_imu.rows));
    cv::addWeighted(img_imu, alpha, disp_rect_imu, beta, gamma, disp_rect_imu);

    // *** Additional info ***
    cv::Mat disp_rect_info = _out_dst(cv::Rect(5, 5, _img.cols - 25, nTextOffsetY * 6));
    cv::Mat black_rect = cv::Mat::zeros(disp_rect_info.rows, disp_rect_info.cols, CV_8UC3);
    cv::addWeighted(black_rect, alpha, disp_rect_info, beta, gamma, disp_rect_info);

    // Print computed extrinsics and body pose
    CImageDisplayUtils::printT(_out_dst, "Abs [W]", _abs_body_pose_W.transform(), 1 * nTextOffsetY, color::blue, font_scale, font_thickness, "euler");

    // Number of keypoints
    snprintf(str, sizeof(str) - 1, "Num keypoints: %zd", curr_inliers_pts.size());
    cv::putText(_out_dst, str, cv::Point(5, 2 * nTextOffsetY), cv::FONT_HERSHEY_PLAIN, font_scale, color::blue, font_thickness);

    // Errors
    float fReprojErr = CProjectiveGeometry::getReprojectionErr(_pSensorModel, prev_inliers_pts, curr_inliers_pts, CPose().transform(), _abs_cam_pose_C.transform());
    snprintf(str, sizeof(str) - 1, "Reprojection error: %.3f", fReprojErr);
    cv::putText(_out_dst, str, cv::Point2f(5, 3 * nTextOffsetY), cv::FONT_HERSHEY_PLAIN, font_scale, color::reprojection, font_thickness);

    // Orthogonal projection to epiplar lines
    float fOrthogonalErr = CProjectiveGeometry::getOrthogonalErr(prev_inliers_pts, curr_inliers_pts, epi_lines_prev, epi_lines_curr);
    snprintf(str, sizeof(str) - 1, "Orthogonal error on epi lines: %.3f", fOrthogonalErr);
    cv::putText(_out_dst, str, cv::Point2f(5, 4 * nTextOffsetY), cv::FONT_HERSHEY_PLAIN, font_scale, color::orthogonal_projection, font_thickness);

    // Scale factor
    snprintf(str, sizeof(str) - 1, "Scale factor: %.2f", _scale_factor);
    cv::putText(_out_dst, str, cv::Point2f(5, 5 * nTextOffsetY), cv::FONT_HERSHEY_PLAIN, font_scale, color::blue, font_thickness);
}

void CImageDisplayUtils::drawSlam(const cv::Mat& _img,
    cv::Mat& _out_dst,
    const CPinholeCameraSensorModel* _pSensorModel,
    const CycSlam& _slam_data,
    const bool _draw_epi_projections,
    const CCycCache* _preintegrated_imu_hist)
{
    /*drawSlam(_img,
        _out_dst,
        _pSensorModel,
        _slam_data.abs_pose_W,
        _slam_data.relative_pose_C,
        _slam_data.keypoints_prev,
        _slam_data.keypoints_curr,
        _slam_data.voxels_prev,
        _slam_data.voxels_curr,
        _slam_data.scale_factor,
        _slam_data.scale_factor_samples_1,
        _slam_data.scale_factor_samples_2,
        _draw_epi_projections,
        _preintegrated_imu_hist);*/
}

void CImageDisplayUtils::drawSlam(const CycImage_& _rimg,
    cv::Mat& _out_dst,
    const CPinholeCameraSensorModel* _pSensorModel,
    const CycSlam& _slam_data,
    const bool _draw_epi_projections,
    const CCycCache* _preintegrated_imu_hist)
{
    cv::Mat img_rgb = cv::Mat(_rimg.nRows, _rimg.nCols, _rimg.nType1, _rimg.pData1);

    /*drawSlam(img_rgb,
        _out_dst,
        _pSensorModel,
        _slam_data.abs_pose_W,
        _slam_data.relative_pose_C,
        _slam_data.keypoints_prev,
        _slam_data.keypoints_curr,
        _slam_data.voxels_prev,
        _slam_data.voxels_curr,
        _slam_data.scale_factor,
        _slam_data.scale_factor_samples_1,
        _slam_data.scale_factor_samples_2,
        _draw_epi_projections,
        _preintegrated_imu_hist);*/
}

void CImageDisplayUtils::drawScaleFactor(cv::Mat& _out_dst,
    const CPinholeCameraSensorModel* _pSensorModel,
    const CPose& _relative_pose_C,
    const CycVoxels& _voxels_prev_corresp,
    const CycVoxels& _voxels_curr_corresp,
    const std::vector<CyC_INT>& _samples_1_ids,
    const std::vector<CyC_INT>& _samples_2_ids)
{
    assert(_samples_1_ids.size() != _samples_2_ids.size());
    assert(_out_dst.empty());

    for (CyC_INT k = 1; k < _samples_1_ids.size(); ++k)
    {

        CyC_INT i = _samples_1_ids[k];
        CyC_INT j = _samples_2_ids[k];

        CycVoxel vx_prev_1, vx_prev_2, vx_curr_1, vx_curr_2;
        CKeypointsMatching::getVoxelById(_voxels_prev_corresp, i, vx_prev_1);
        CKeypointsMatching::getVoxelById(_voxels_prev_corresp, j, vx_prev_2);
        CKeypointsMatching::getVoxelById(_voxels_curr_corresp, i, vx_curr_1);
        CKeypointsMatching::getVoxelById(_voxels_curr_corresp, j, vx_curr_2);

        // Project voxels
        CycPoint pt_prev_1 = CProjectiveGeometry::project(_pSensorModel, CPose().transform(), vx_prev_1);
        CycPoint pt_prev_2 = CProjectiveGeometry::project(_pSensorModel, CPose().transform(), vx_prev_2);
        CycPoint pt_curr_1 = CProjectiveGeometry::project(_pSensorModel, _relative_pose_C.transform(), vx_curr_1);
        CycPoint pt_curr_2 = CProjectiveGeometry::project(_pSensorModel, _relative_pose_C.transform(), vx_curr_2);

        cv::Point2f cv_pt_prev_1{ pt_prev_1.pt2d.x(), pt_prev_1.pt2d.y() };
        cv::Point2f cv_pt_prev_2{ pt_prev_2.pt2d.x(), pt_prev_2.pt2d.y() };
        cv::Point2f cv_pt_curr_1{ pt_curr_1.pt2d.x() + CyC_INT(_out_dst.cols / 2.f), pt_curr_1.pt2d.y() };
        cv::Point2f cv_pt_curr_2{ pt_curr_2.pt2d.x() + CyC_INT(_out_dst.cols / 2.f), pt_curr_2.pt2d.y() };

        cv::circle(_out_dst, cv_pt_prev_1, 6, CV_RGB(255, 20, 28), 2);
        cv::circle(_out_dst, cv_pt_prev_2, 6, CV_RGB(255, 20, 28), 2);
        cv::circle(_out_dst, cv_pt_curr_1, 6, CV_RGB(255, 20, 28), 2);
        cv::circle(_out_dst, cv_pt_curr_2, 6, CV_RGB(255, 20, 28), 2);
        cv::line(_out_dst, cv_pt_prev_1, cv_pt_prev_2, CV_RGB(255, 20, 28), 2);
        cv::line(_out_dst, cv_pt_curr_1, cv_pt_curr_2, CV_RGB(255, 20, 28), 2);
    }
}

void CImageDisplayUtils::drawPreintegratedImu(cv::Mat& _out_dst, const CCycCache* _preintegrated_imu_hist)
{
    if (_preintegrated_imu_hist == nullptr)
        return;

    bool bFirst = true;
    CyC_INT y_scale = 5;
    CyC_INT x = 0;
    
    cv::Point cv_pt_prev_acc_x(x, 0);
    cv::Point cv_pt_prev_acc_y(x, 0);
    cv::Point cv_pt_prev_acc_z(x, 0);

    for (CyC_INT n : _preintegrated_imu_hist->keys())
    {
        CyC_INT y_acc_x = (_out_dst.rows / 2) - ((CyC_INT)_preintegrated_imu_hist->at<CycImu>(n).acc.x() * y_scale);
        CyC_INT y_acc_y = (_out_dst.rows / 2) - ((CyC_INT)_preintegrated_imu_hist->at<CycImu>(n).acc.y() * y_scale);
        CyC_INT y_acc_z = (_out_dst.rows / 2) - ((CyC_INT)_preintegrated_imu_hist->at<CycImu>(n).acc.z() * y_scale);

        cv::Point cv_pt_curr_acc_x(x, y_acc_x);
        cv::Point cv_pt_curr_acc_y(x, y_acc_y);
        cv::Point cv_pt_curr_acc_z(x, y_acc_z);
        
        if (!bFirst)
        {
            cv::line(_out_dst, cv_pt_prev_acc_x, cv_pt_curr_acc_x, color::x_axis, 1);
            cv::line(_out_dst, cv_pt_prev_acc_y, cv_pt_curr_acc_y, color::y_axis, 1);
            cv::line(_out_dst, cv_pt_prev_acc_z, cv_pt_curr_acc_z, color::z_axis, 1);
        }
        bFirst = false;

        cv_pt_prev_acc_x = cv_pt_curr_acc_x;
        cv_pt_prev_acc_y = cv_pt_curr_acc_y;
        cv_pt_prev_acc_z = cv_pt_curr_acc_z;

        ++x;
    }

    double font_scale = _out_dst.rows / 200.;
    cv::putText(_out_dst, "inertial datastream", cv::Point(5, _out_dst.rows - 8), cv::FONT_HERSHEY_PLAIN, font_scale, CV_RGB(255, 255, 255), 1);
}

void CImageDisplayUtils::draw_slam_grid(cv::Mat& _out_dst, const cv::Size& _grid_size)
{
    CyC_INT disp_scale = 2;
    if (_grid_size.width <= 400 || _grid_size.height <= 300)
        disp_scale = 5;

    _out_dst = cv::Mat(cv::Size(_grid_size.width * disp_scale, _grid_size.height * disp_scale), CV_8UC3, cv::Scalar(150, 150, 150));

    // Draw the grid's x axis
    CyC_UINT w = 20;
    for (CyC_UINT i = 0; i < static_cast<CyC_UINT>(_out_dst.cols); i += 3)
    {
        cv::Point pt1 = cv::Point((CyC_UINT)i * w, 0);
        cv::Point pt2 = cv::Point((CyC_UINT)i * w, _out_dst.rows - 1);
        cv::line(_out_dst, pt1, pt2, CV_RGB(0, 0, 0));
    }

    // Draw the grid's y axis
    CyC_UINT h = 20;
    for (CyC_UINT i = 0; i < static_cast<CyC_UINT>(_out_dst.rows); i += 3)
    {
        cv::Point pt1 = cv::Point(0, (CyC_UINT)i * h);
        cv::Point pt2 = cv::Point(_out_dst.cols - 1, (CyC_UINT)i * h);
        cv::line(_out_dst, pt1, pt2, CV_RGB(0, 0, 0));
    }

    // Draw origin
    cv::Point2f pt_or = cv::Point(_out_dst.cols / 2.f, _out_dst.rows / 2.f);
    cv::line(_out_dst, cv::Point2f(pt_or.x - 10, pt_or.y), cv::Point2f(pt_or.x + 10, pt_or.y), color::red, 1);
    cv::line(_out_dst, cv::Point2f(pt_or.x, pt_or.y - 10), cv::Point2f(pt_or.x, pt_or.y + 10), color::red, 1);
}

void CImageDisplayUtils::draw_slam(cv::Mat& _out_dst,
    const cv::Size& _grid_size,
    const CycSlam& _slam_data,
    const float& _bew_scale,
    const bool& _absolute_coord,
    const bool& _draw_curr_frame,
    const bool& _show_curr_data)
{
    if (m_pSlamDispCache == nullptr)
        m_pSlamDispCache = std::make_unique<CCycCache>(100);
    
    char str[128];
    CyC_INT nTextOffsetY = _grid_size.height / 10;
    double alpha = 0.85;
    double beta = 0.28;
    double gamma = 0.;
    double font_scale = 1.8;
    CyC_INT font_thickness = 2;

    draw_slam_grid(_out_dst, _grid_size);
    cv::Point2f pt_or = cv::Point(_out_dst.cols / 2.f, _out_dst.rows / 2.f);

    // Used for relative coordinates drawing
    CPose rel_pose(_slam_data.Absolute_Body_W.translation_3x1().x(),
        _slam_data.Absolute_Body_W.translation_3x1().y(),
        _slam_data.Absolute_Body_W.translation_3x1().z(),
        0.f, 0.f, 0.f);
    rel_pose = rel_pose.inverse();

    // Draw map points
    for (size_t i = 0; i < _slam_data.rel_map_points_W.size(); ++i)
    {
        cv::Point2f pt;
        Eigen::Vector4f MP = _slam_data.Absolute_Body_W.transform() * _slam_data.rel_map_points_W[i].first.pt3d;

        if (!_absolute_coord)
        {
            CycVoxel vx_rel_W{ rel_pose.transform() * MP };
            pt.x = vx_rel_W.pt3d.x();
            pt.y = -vx_rel_W.pt3d.y();
        }
        else
        {
            pt.x = MP.x();
            pt.y = -MP.y();
        }

        // Center on origin and scale
        pt.x = pt_or.x + pt.x * _bew_scale;
        pt.y = pt_or.y + pt.y * _bew_scale;

        cv::circle(_out_dst, pt, 1, color::white, 2);

        snprintf(str, sizeof(str) - 1, "%d", _slam_data.rel_map_points_W[i].first.id);
        cv::putText(_out_dst, str, pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 1., color::blue, 2);
    }

    // Draw robot trajectory
    //m_SlamTrajectory.emplace_back(_slam_data.Absolute_Body_W.translation_3x1());
    //for (size_t i = 1; i < m_SlamTrajectory.size(); ++i)
    //{
    //    Eigen::Vector2f pt_prev, pt_curr;

    //    if (!_absolute_coord)
    //    {
    //        Eigen::Vector4f hom_prev = Eigen::Vector4f(m_SlamTrajectory[i - 1].x(), m_SlamTrajectory[i - 1].y(), m_SlamTrajectory[i - 1].z(), 1.f);
    //        Eigen::Vector4f hom_curr = Eigen::Vector4f(m_SlamTrajectory[i].x(), m_SlamTrajectory[i].y(), m_SlamTrajectory[i].z(), 1.f);
    //        Eigen::Vector4f vx_rel_W_prev = rel_pose.transform() * hom_prev;
    //        Eigen::Vector4f vx_rel_W_curr = rel_pose.transform() * hom_curr;

    //        pt_prev.x() = vx_rel_W_prev.x();
    //        pt_prev.y() = -vx_rel_W_prev.y();

    //        pt_curr.x() = vx_rel_W_curr.x();
    //        pt_curr.y() = -vx_rel_W_curr.y();
    //    }
    //    else
    //    {
    //        pt_prev.x() = m_SlamTrajectory[i - 1].x();
    //        pt_prev.y() = -m_SlamTrajectory[i - 1].y();

    //        pt_curr.x() = m_SlamTrajectory[i].x();
    //        pt_curr.y() = -m_SlamTrajectory[i].y();
    //    }

    //    // Center on origin and scale
    //    pt_prev.x() = pt_or.x + pt_prev.x() * _bew_scale;
    //    pt_prev.y() = pt_or.y + pt_prev.y() * _bew_scale;
    //    pt_curr.x() = pt_or.x + pt_curr.x() * _bew_scale;
    //    pt_curr.y() = pt_or.y + pt_curr.y() * _bew_scale;

    //    cv::line(_out_dst, cv::Point2f(pt_prev.x(), pt_prev.y()), cv::Point2f(pt_curr.x(), pt_curr.y()), color::current_path, 4);
    //}

    // Draw stored slam data
    Eigen::Vector2f prev_cam_position;
    for (size_t i = 0; i < _slam_data.prev_poses_Body_W.size(); ++i)
    {
        cv::Scalar color = color::red;
        const CPose prev_body_W = _slam_data.prev_poses_Body_W[i].second;
        
        // Calculate offset from the current pose
        Eigen::Vector2f cam_position;
        if (_absolute_coord)
            cam_position = Eigen::Vector2f(prev_body_W.translation_3x1().x(), prev_body_W.translation_3x1().y());
        else
            cam_position = Eigen::Vector2f(prev_body_W.translation_3x1().x() - _slam_data.Absolute_Body_W.translation_3x1().x(),
                prev_body_W.translation_3x1().y() - _slam_data.Absolute_Body_W.translation_3x1().y());

        // Check if the frame is a local frame
        if (_slam_data.prev_poses_type[i] == 1)
            color = color::green;

        // Check if the frame is a neighboring frame
        if (_slam_data.prev_poses_type[i] == 2)
            color = color::yellow;

        if (i == 0)
        {
            Eigen::Vector2f curr_cam_pose = Eigen::Vector2f::Zero();
            if (_absolute_coord)
                curr_cam_pose = Eigen::Vector2f(_slam_data.Absolute_Body_W.translation_3x1().x(), _slam_data.Absolute_Body_W.translation_3x1().y());
            
            cv::line(_out_dst,
                cv::Point2f(pt_or.x + cam_position.x() * _bew_scale, pt_or.y - cam_position.y() * _bew_scale),
                cv::Point2f(pt_or.x + curr_cam_pose.x() * _bew_scale, pt_or.y - curr_cam_pose.y() * _bew_scale),
                color::current_path, 1);
        }
        else
        {
            cv::line(_out_dst, 
                cv::Point2f(pt_or.x + prev_cam_position.x() * _bew_scale, pt_or.y - prev_cam_position.y() * _bew_scale),
                cv::Point2f(pt_or.x + cam_position.x() * _bew_scale, pt_or.y - cam_position.y() * _bew_scale),
                color::current_path, 1);
        }
        CImageDisplayUtils::draw_pose(_out_dst, cam_position.x(), cam_position.y(), prev_body_W.rotation_euler().z(), _bew_scale, color, 2);

        //if (_slam_data.prev_poses_Body_W[i].first == 44)
        //    draw_pose(_out_dst, cam_position.x(), cam_position.y(), prev_body_W.rotation_euler().z(), _bew_scale, color::red, 2);

        prev_cam_position = cam_position;
    }

    // Draw current slam data
    if (_draw_curr_frame)
    {
        // Draw body frame
        if (_absolute_coord)
            CImageDisplayUtils::draw_pose(_out_dst, _slam_data.Absolute_Body_W.translation_3x1().x(),
                _slam_data.Absolute_Body_W.translation_3x1().y(),
                _slam_data.Absolute_Body_W.rotation_euler().z(), _bew_scale, color::blue, 4);
        else
            CImageDisplayUtils::draw_pose(_out_dst, 0.f, 0.f, _slam_data.Absolute_Body_W.rotation_euler().z(), _bew_scale, color::blue, 4);

        // Draw IMU frame
        if (_absolute_coord)
            CImageDisplayUtils::draw_pose(_out_dst, _slam_data.Absolute_Imu_W.translation_3x1().x(),
                _slam_data.Absolute_Imu_W.translation_3x1().y(),
                _slam_data.Absolute_Imu_W.rotation_euler().z(), _bew_scale, color::pink, 4);
        else
            CImageDisplayUtils::draw_pose(_out_dst, 0.f, 0.f, _slam_data.Absolute_Imu_W.rotation_euler().z(), _bew_scale, color::pink, 4);
    }

    // *** Additional info ***
    if (_show_curr_data)
    {
        cv::Mat disp_rect_info = _out_dst(cv::Rect(5, 5, _out_dst.cols - 20, CyC_INT(nTextOffsetY * 3.5)));
        cv::Mat black_rect = cv::Mat::zeros(disp_rect_info.rows, disp_rect_info.cols, CV_8UC3);
        cv::addWeighted(black_rect, alpha, disp_rect_info, beta, gamma, disp_rect_info);

        ////cv::resize(_out_dst, _out_dst, cv::Size(_out_dst.cols * 2, _out_dst.rows * 2));

        // Print computed body pose
        CImageDisplayUtils::printT(_out_dst, "Body [W]", _slam_data.Absolute_Body_W.transform(), nTextOffsetY, color::info, font_scale, font_thickness, "euler");

        // Print velocity
        snprintf(str, sizeof(str) - 1, "V [%.2f; %.2f; %.2f]", _slam_data.Velocity_W.x(), _slam_data.Velocity_W.y(), _slam_data.Velocity_W.z());
        cv::putText(_out_dst, str, cv::Point(_out_dst.cols - nTextOffsetY * 11.5, nTextOffsetY), cv::FONT_HERSHEY_PLAIN, font_scale, color::info, font_thickness);

        // Print bias
        snprintf(str, sizeof(str) - 1, "Bias [I]: acc [%.2f; %.2f; %.2f]  gyro [%.2f; %.2f; %.2f]",
            _slam_data.Bias_Acc_I.x(), _slam_data.Bias_Acc_I.y(), _slam_data.Bias_Acc_I.z(),
            _slam_data.Bias_Gyro_I.x(), _slam_data.Bias_Gyro_I.y(), _slam_data.Bias_Gyro_I.z());
        cv::putText(_out_dst, str, cv::Point(5, nTextOffsetY * 2), cv::FONT_HERSHEY_PLAIN, font_scale, color::info, font_thickness);

        // Keypoints information
        snprintf(str, sizeof(str) - 1, "Map matches: %zd; Map points: %d; KeyFrames: %d",
            _slam_data.rel_map_points_W.size(), _slam_data.num_map_points, _slam_data.num_keyframes);
        cv::putText(_out_dst, str, cv::Point(5, nTextOffsetY * 3), cv::FONT_HERSHEY_PLAIN, font_scale, color::info, font_thickness);

        // Draw axes convention
        cv::Point ptOrg = cv::Point(50, _out_dst.rows - 50);
        cv::Point ptX = cv::Point(100, _out_dst.rows - 50);
        cv::Point ptY = cv::Point(50, _out_dst.rows - 100);
        cv::arrowedLine(_out_dst, ptOrg, ptX, color::x_axis, 2);
        cv::arrowedLine(_out_dst, ptOrg, ptY, color::y_axis, 2);
        cv::circle(_out_dst, ptOrg, 8, color::z_axis, -1);
        cv::putText(_out_dst, "X", ptX, cv::FONT_HERSHEY_PLAIN, font_scale, color::x_axis, font_thickness);
        cv::putText(_out_dst, "Y", ptY, cv::FONT_HERSHEY_PLAIN, font_scale, color::y_axis, font_thickness);
    }
    
    if (_slam_data.is_keyframe)
        m_pSlamDispCache->insert(_slam_data.timestamp, _slam_data);
}

void CImageDisplayUtils::draw_slam_frame(cv::Mat& _out_dst, const CPinholeCameraSensorModel* _pCamSensorModel, const CycSlam& _slam_data)
{
    float disp_scale = 1;
    if (_pCamSensorModel->width() < 400 || _pCamSensorModel->height() < 300)
        disp_scale = 2;

    char str[128];
    
    //_out_dst = cv::Mat(_frame->m_rImg.nRows, _frame->m_rImg.nCols, _frame->m_rImg.nType1, _frame->m_rImg.pData1).clone();
    cv::resize(_out_dst, _out_dst, cv::Size(_out_dst.cols * disp_scale, _out_dst.rows * disp_scale));

    // Reproject map points
    CPose abs_cam_C = _pCamSensorModel->extrinsics_inv() * _slam_data.Absolute_Body_W * _pCamSensorModel->extrinsics();

    CycPoints vxs_reproj;
    //CProjectiveGeometry::project(m_pCamSensorModel, m_Absolute_Cam_C.transform(), m_pMap->m_MapPoints, vxs_reproj);
    for (const auto& MP : _slam_data.rel_map_points_W)
    {
        CycVoxel vx_C = MP.first;
        vx_C.pt3d = _slam_data.Absolute_Body_W.transform() * vx_C.pt3d;
        vx_C.pt3d = _pCamSensorModel->extrinsics_inv().transform() * vx_C.pt3d;
        vxs_reproj.emplace_back(CProjectiveGeometry::project(_pCamSensorModel, abs_cam_C.transform(), vx_C));
    }

    // Compute projection matrices (required for computing the depth)
    Pmatrix P_cam = CProjectiveGeometry::KT2P(_pCamSensorModel->K(), CProjectiveGeometry::invertT(abs_cam_C.transform()));

    // Reproject matched map points to 2D for visualization
    for (size_t k = 0; k < _slam_data.rel_map_points_W.size(); ++k)
    {
        const CycVoxel* pMP = &_slam_data.rel_map_points_W[k].first;

        cv::Point2f cv_pt_curr_reproj((vxs_reproj[k].pt2d.x() / _pCamSensorModel->width()) * _out_dst.cols,
            (vxs_reproj[k].pt2d.y() / _pCamSensorModel->height()) * _out_dst.rows);

        // Calculate depth in both projections
        CycVoxel vx_C = *pMP;
        vx_C.pt3d = _slam_data.Absolute_Body_W.transform() * vx_C.pt3d;
        vx_C.pt3d = _pCamSensorModel->extrinsics_inv().transform() * vx_C.pt3d;
        float depth_curr = CTriangulation::getDepth(vx_C, P_cam);

        // Draw observation
        cv::Point2f cv_pt2d_1(_slam_data.rel_map_points_W[k].second.pt2d.x() * _out_dst.cols, _slam_data.rel_map_points_W[k].second.pt2d.y() * _out_dst.rows);
        cv::circle(_out_dst, cv_pt2d_1 * disp_scale, 1, color::orthogonal_projection, 6);

        cv::line(_out_dst, cv_pt2d_1 * disp_scale, cv_pt_curr_reproj * disp_scale, color::green, 1);

        // Draw map point reprojection
        snprintf(str, sizeof(str) - 1, "%d", pMP->id);
        if (depth_curr > 0.f)
        {
            cv::circle(_out_dst, cv_pt_curr_reproj * disp_scale, 7, color::reproj_pts_2d_pos_depth, 2);
            cv::putText(_out_dst, str, cv_pt_curr_reproj * disp_scale, cv::FONT_HERSHEY_PLAIN, 0.95, color::blue, 1);
        }
        else
        {
            cv::circle(_out_dst, cv_pt_curr_reproj * disp_scale, 7, color::reproj_pts_2d_neg_depth, 2);
            cv::putText(_out_dst, str, cv_pt_curr_reproj * disp_scale, cv::FONT_HERSHEY_PLAIN, 1.15, color::reproj_pts_2d_neg_depth, 2);
        }

        // Check if this is a new voxel added to the map
        for (const auto& idx : _slam_data.latest_map_points)
            if (pMP->id == idx)
                cv::circle(_out_dst, cv_pt_curr_reproj * disp_scale, 10, color::black, 1);

        // Draw reprojection error
        //cv::line(_out_dst, cv_pt_curr, cv_pt_curr_reproj, color::reprojection, 1);
    }

    snprintf(str, sizeof(str) - 1, "Frame ID: %lld", static_cast<long long>(_slam_data.id));
    cv::putText(_out_dst, str, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, disp_scale, CV_RGB(255, 255, 255), 2);

    double alpha = 0.85;
    double beta = 0.28;
    double gamma = 0.;
    double font_scale = 1.3;
    CyC_INT font_thickness = 2;
    CyC_INT nTextOffsetY = _pCamSensorModel->height() / 11;
    cv::Mat disp_rect_info = _out_dst(cv::Range(_out_dst.rows - CyC_INT(nTextOffsetY * 1.5), _out_dst.rows - 5), cv::Range(5, CyC_INT(nTextOffsetY * 4)));
    cv::Mat black_rect = cv::Mat::zeros(disp_rect_info.rows, disp_rect_info.cols, CV_8UC3);
    cv::addWeighted(black_rect, alpha, disp_rect_info, beta, gamma, disp_rect_info);

    if (_slam_data.is_mapping)
        cv::putText(_out_dst, "Mapping", cv::Point(15, _out_dst.rows - CyC_INT(nTextOffsetY / 1.5)), cv::FONT_HERSHEY_PLAIN, font_scale, color::red, font_thickness);
    else
        cv::putText(_out_dst, "Tracking", cv::Point(15, _out_dst.rows - CyC_INT(nTextOffsetY / 1.5)), cv::FONT_HERSHEY_PLAIN, font_scale, color::green, font_thickness);

    ////// *** Draw correspondences ***
    ////cv::Mat img_corresp;
    ////cv::Mat img_corresp_prev = cv::Mat::zeros(_out_dst.rows, _out_dst.cols, CV_8UC3);
    ////cv::Mat img_corresp_curr = cv::Mat::zeros(_out_dst.rows, _out_dst.cols, CV_8UC3);

    ////// TODO: fix dummy inliers
    ////CycPoints ransacInliers_prev, ransacInliers_curr;
    ////CImageDisplayUtils::draw_correspondences(img_corresp_prev, img_corresp_curr, img_corresp, ransacInliers_prev, ransacInliers_curr, 2);

    ////// Draw the sampled voxels used for calculating the scale factor, if any
    //////drawScaleFactor(img_corresp, _pSensorModel, _relative_cam_pose_C, voxels_prev_C, voxels_curr_C, _scale_factor_samples_1, _scale_factor_samples_2);

    ////// Stich correspondences image to the main display
    ////cv::resize(img_corresp, img_corresp, cv::Size(img_corresp.cols / 3.f, img_corresp.rows / 3.f));
    ////cv::Mat disp_rect_corresp = _out_dst(cv::Rect(0, _out_dst.rows - img_corresp.rows, img_corresp.cols, img_corresp.rows));
    ////cv::addWeighted(img_corresp, alpha, disp_rect_corresp, beta, gamma, disp_rect_corresp);
}