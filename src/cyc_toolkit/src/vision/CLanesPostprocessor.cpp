#include "CLanesPostprocessor.h"

#define USE_NEW_DBSCAN 1

#ifdef USE_NEW_DBSCAN
#include "clustering/dbscan_new.hpp"
#endif 

void CLanesPostprocessor::cluster_lanes(const cv::Mat& tf_binary_seg_output, const cv::Mat& tf_instance_seg_output,
	std::vector<cv::Point>& coords_ret, std::vector<std::vector<uint>>& clusters)
{
	_pix_embedding_feature_dims = 4;
	_dbscan_core_object_min_pts = 75;
	_dbscan_neighbor_radius = 0.4f;
	_embedding_feats_dilution_ratio = 0.1f;


	cv::Mat img_binary_seg;
	cv::Mat img_instance_seg;

	//tf_binary_seg_output *= 255;
	tf_binary_seg_output.convertTo(img_binary_seg, CV_8UC1);
	//img_binary_seg *= 255;
	tf_instance_seg_output.convertTo(img_instance_seg, CV_32FC4);

	std::vector<cv::Point> coords;
	std::vector<DBSCANSample<float>> pixel_embedding_samples;
	gather_pixel_embedding_features(img_binary_seg, img_instance_seg, coords, pixel_embedding_samples);

	// simultaneously random shuffle embedding vector and coord vector inplace
	simultaneously_random_shuffle(coords, pixel_embedding_samples);

	// simultaneously random select embedding vector and coord vector to reduce the cluster time
	std::vector<DBSCANSample<float>> pixel_embedding_samples_selected;
	coords_ret.clear();
	simultaneously_random_select(pixel_embedding_samples, coords, this->_embedding_feats_dilution_ratio, pixel_embedding_samples_selected, coords_ret);

	// normalize pixel embedding features
	normalize_sample_features(pixel_embedding_samples_selected, pixel_embedding_samples_selected);

	// cluster samples
	std::vector<uint> noise;

	clusters.clear();
	cluster_pixem_embedding_features(pixel_embedding_samples_selected, clusters, noise);
}

void CLanesPostprocessor::clusters2lanes(
	const std::vector<cv::Point>& coords_selected,
	const std::vector<std::vector<uint>>& cluster_ret,
	CycLanesModel& lanes_model)
{
	lanes_model.clear();

	for (CyC_UINT lane_id = 0; lane_id < cluster_ret.size(); ++lane_id)
	{
		std::vector< Eigen::Vector2f > lane_pts;
		for (auto index = 0; index < cluster_ret[lane_id].size(); ++index)
		{
			cv::Point coord = coords_selected[cluster_ret[lane_id][index]];
			lane_pts.emplace_back(coord.x, coord.y);
		}

		Eigen::VectorXf xvals(lane_pts.size());
		Eigen::VectorXf yvals(lane_pts.size());

		for (size_t i = 0; i < lane_pts.size(); ++i)
		{
			xvals(i) = lane_pts[i].x();
			yvals(i) = lane_pts[i].y();
		}

		lanes_model.emplace_back(lane_id, CPolynomialFitting::polyfit(xvals, yvals, 3));
	}
}

void CLanesPostprocessor::scale_lanes(const cv::Size& org_img_size, const cv::Size& inference_img_size, CycLanesModel& lanes_model
)
{
	float scale_x = (float)org_img_size.width / inference_img_size.width;
	float scale_y = (float)org_img_size.height / inference_img_size.height;

	for (auto& lane : lanes_model)
	{
		bool added = false;
		std::vector<Eigen::Vector2f> lane_pts;

		for (float x = 0; x <= inference_img_size.width; x += 3.f)
		{
			float y = CPolynomialFitting::polyeval(lane.model, x);

			if (y > inference_img_size.height / 1.8f)
			{
				cv::Point2f coord{ x, y };
				coord.x *= scale_x;
				coord.y *= scale_y;

				lane_pts.emplace_back(coord.x, coord.y);

				added = true;
			}
			else if (added)
				break;
		}

		Eigen::VectorXf xvals(lane_pts.size());
		Eigen::VectorXf yvals(lane_pts.size());

		for (size_t i = 0; i < lane_pts.size(); ++i)
		{
			xvals(i) = lane_pts[i].x();
			yvals(i) = lane_pts[i].y();
		}

		lane.model = CPolynomialFitting::polyfit(xvals, yvals, 3);
	}
}

CycLane CLanesPostprocessor::lane2world(const CycLane& lane, const cv::Point& org_img_size, const CPinholeCameraSensorModel& camera_model)
{
	Eigen::Matrix4f M_cam2veh = camera_model.pose().transform();
	float Y = camera_model.pose().translation_3x1().z();
	std::vector< Eigen::Vector4f > lane_pts_3d;

	bool added = false;
	for (float x = 0; x <= org_img_size.x; x += 3.f)
	{
		float y = CPolynomialFitting::polyeval(lane.model, x);

		if (y > org_img_size.y / 1.8f)
		{
			cv::Point2f coord{ x, y };

			if ((coord.y - camera_model.cy()) != 0)
			{
				float Z = (camera_model.fy_px() * Y) / (coord.y - camera_model.cy());
				float X = (Z * (coord.x - camera_model.cx())) / camera_model.fx_px();

				Eigen::Vector4f pt3dCam = Eigen::Vector4f{ X, Y, Z, 1 };

				// Map 3D coordinates in camera frame to 3D coordinates in the vehicle frame
				Eigen::Vector4f pt3dVeh = M_cam2veh * pt3dCam;
				lane_pts_3d.push_back(pt3dVeh);
			}

			added = true;
		}
		else if (added)
			break;
	}

	Eigen::VectorXf xvals(lane_pts_3d.size());
	Eigen::VectorXf yvals(lane_pts_3d.size());

	for (auto i = 0; i < lane_pts_3d.size(); ++i)
	{
		xvals(i) = lane_pts_3d[i].x();
		yvals(i) = lane_pts_3d[i].y();
	}

	return CycLane(lane.id, CPolynomialFitting::polyfit(xvals, yvals, 3));
}

void CLanesPostprocessor::map_clusters_to_lanes_model(const std::vector<cv::Point>& coords_selected,
	const std::vector<std::vector<uint>>& cluster_ret, const cv::Size& inference_img_size, CPinholeCameraSensorModel& camera_model, CycLanesModel& lanes_model)
{
	// TBD: remove this hardcode
	//CFrames frames;
	//Eigen::Matrix4f M_veh2cam = frames.transform_mat_ZXY(0.0159f, 1.5f, 1.7f, 0.f * DEG2RAD, 270.f * DEG2RAD, 90.f * DEG2RAD);
	//M_veh2cam << 0.f, -1.f, -0.f, 0.0159f,
	//	0.f, 0.f, -1.f, 1.5f,
	//	1.f, 0.f, 0.f, 1.7f,
	//	0.f, 0.f, 0.f, 1.f;
	//Eigen::Matrix4f M_cam2veh = M_veh2cam.inverse();
	//float Y = 1.5f;
	// TBD: remove this hardcode

	std::vector< std::vector< Eigen::Vector4f > > lanes_pts_3d_veh;

	Eigen::Matrix4f M_cam2veh = camera_model.pose().transform();
	float Y = camera_model.pose().translation_3x1().z();
	float scale_cam_img_x = (float)camera_model.width() / (float)inference_img_size.width;
	float scale_cam_img_y = (float)camera_model.height() / (float)inference_img_size.height;

	for (CyC_UINT lane_id = 0; lane_id < cluster_ret.size(); ++lane_id)
	{
		std::vector< Eigen::Vector4f > lane_pts_3d;
		for (auto index = 0; index < cluster_ret[lane_id].size(); ++index)
		{
			// Get pixel coordinates in 2D image
			cv::Point coord = coords_selected[cluster_ret[lane_id][index]];
			coord.x *= scale_cam_img_x;
			coord.y *= scale_cam_img_y;

			// Calculate 3D coordinates in camera frame
			if ((coord.y - camera_model.cy()) != 0)
			{
				float Z = (camera_model.fy_px() * Y) / (coord.y - camera_model.cy());
				float X = (Z * (coord.x - camera_model.cx())) / camera_model.fx_px();

				Eigen::Vector4f pt3dCam = Eigen::Vector4f{ X, Y, Z, 1 };

				// Map 3D coordinates in camera frame to 3D coordinates in the vehicle frame
				Eigen::Vector4f pt3dVeh = M_cam2veh * pt3dCam;
				lane_pts_3d.push_back(pt3dVeh);

				// Debug
				//Eigen::Matrix4f M_veh2cam = camera_model.getM_baseframe2sensor();
				//Eigen::Vector4f pt3dCam_reproj = M_veh2cam * pt3dVeh;
				//Eigen::Vector2f pt2D = pSensorModel.world2sensor(pt3dCam_reproj);
				//Eigen::Vector2f pt2D_reproj = pSensorModel.world2sensor(pt3dCam);
				//cv::circle(img_viz, cv::Point2f{ pt2D.x(), pt2D.y() }, 3, CV_RGB(255, 255, 0), -1);
				//cv::circle(img_viz, cv::Point2f{ pt2D_reproj.x(), pt2D_reproj.y() }, 3, CV_RGB(255, 100, 0), 2);
				//cv::circle(img_viz, cv::Point{ coord.x, coord.y }, 3, CV_RGB(0, 100, 255), -1);
			}
		}
		lanes_pts_3d_veh.push_back(lane_pts_3d);
	}

	// Lanes model estimation
	lanes_model.clear();
	for (auto lane_id = 0; lane_id < lanes_pts_3d_veh.size(); ++lane_id)
	{
		Eigen::VectorXf xvals(lanes_pts_3d_veh[lane_id].size());
		Eigen::VectorXf yvals(lanes_pts_3d_veh[lane_id].size());

		for (auto i = 0; i < lanes_pts_3d_veh[lane_id].size(); ++i)
		{
			xvals(i) = lanes_pts_3d_veh[lane_id][i].x();
			yvals(i) = lanes_pts_3d_veh[lane_id][i].y();
		}

		CycLane lane(lane_id, CPolynomialFitting::polyfit(xvals, yvals, 3));
		lanes_model.push_back(lane);
	}
}
//
//void CLanesPostprocessor::map_clusters_to_2D_lanes_model(const std::vector<cv::Point>& coords_selected,
//	const std::vector<std::vector<uint>>& cluster_ret, const cv::Size& inference_img_size, CycLanesModel& lanes_model)
//{
//
//}
//
//void CLanesPostprocessor::map_clusters_to_2D_lanes_model(CPinholeCameraSensorModel& camera_model, CycLanesModel& lanes_model)
//{
//
//}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CLanesPostprocessor::gather_pixel_embedding_features(const cv::Mat& binary_mask, const cv::Mat& pixel_embedding,
	std::vector<cv::Point>& coords, std::vector<DBSCANSample<float>>& embedding_samples)
{

	auto image_rows = binary_mask.rows;
	auto image_cols = binary_mask.cols;

	for (auto row = 0; row < image_rows; ++row)
	{
		auto binary_image_row_data = binary_mask.ptr<uchar>(row);
		auto embedding_image_row_data = pixel_embedding.ptr<cv::Vec4f>(row);

		for (auto col = 0; col < image_cols; ++col)
		{
			auto binary_image_pix_value = binary_image_row_data[col];
			if (binary_image_pix_value == 255)
			{
				coords.emplace_back(cv::Point(col, row));
				Feature<float> embedding_features;

				for (auto index = 0; index < 4; ++index)
				{
					embedding_features.push_back(embedding_image_row_data[col][index]);
				}

				DBSCANSample sample(embedding_features, CLASSIFY_FLAGS::NOT_CALSSIFIED);
				embedding_samples.push_back(sample);
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CLanesPostprocessor::cluster_pixem_embedding_features(std::vector<DBSCANSample<float>>& embedding_samples,
	std::vector<std::vector<uint> >& cluster_ret, std::vector<uint>& noise)
{
	if (embedding_samples.empty())
	{
		std::cout << "Pixel embedding samples empty";
		return;
	}

	// dbscan cluster
	auto dbscan = DBSCAN<DBSCANSample<float>, float>();

#ifdef USE_NEW_DBSCAN
	std::vector<std::pair<float, float>> tmp;
	for (const auto& s : embedding_samples)
	{
		std::pair<float, float> p = { s.get_feature_vector()[0], s.get_feature_vector()[1] };
		tmp.push_back(p);
	}

	auto ret = NewDBScan::run(tmp, this->_dbscan_neighbor_radius, this->_dbscan_core_object_min_pts);
	size_t idx = 0;
	for (const auto& r : ret)
	{
		cluster_ret.emplace_back();
		for (const auto& r1 : r)
		{
			cluster_ret[idx].emplace_back(r1);
		}

		++idx;
	}
#else
	dbscan.Run(&embedding_samples, _pix_embedding_feature_dims, _dbscan_neighbor_radius, _dbscan_core_object_min_pts);
	cluster_ret = dbscan.Clusters;
	noise = dbscan.Noise;
#endif // USE_NEW_DBSCAN
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CLanesPostprocessor::visualize_instance_segmentation_result(const std::vector<std::vector<uint> >& cluster_ret,
	const std::vector<cv::Point>& coords, cv::Mat& instance_seg_result, float tensor_w, float tensor_h)
{

	auto width = instance_seg_result.cols / tensor_w;
	auto height = instance_seg_result.rows / tensor_h;
	std::map<int, cv::Scalar> color_map =
	{
		{0, cv::Scalar(0, 0, 255)},
		{1, cv::Scalar(0, 255, 0)},
		{2, cv::Scalar(255, 0, 0)},
		{3, cv::Scalar(255, 0, 255)},
		{4, cv::Scalar(0, 255, 255)},
		{5, cv::Scalar(255, 255, 0)},
		{6, cv::Scalar(125, 0, 125)},
		{7, cv::Scalar(0, 125, 125)}
	};

	//omp_set_num_threads(64);
	for (unsigned long class_id = 0; class_id < cluster_ret.size(); ++class_id)
	{
		auto class_color = color_map[class_id];
#pragma omp parallel for
		for (auto index = 0; index < cluster_ret[class_id].size(); ++index)
		{
			auto coord = coords[cluster_ret[class_id][index]];
			coord.x *= width;
			coord.y *= height;
			auto image_col_data = instance_seg_result.ptr<cv::Vec3b>(coord.y);
			image_col_data[coord.x][0] = class_color[0];
			image_col_data[coord.x][1] = class_color[1];
			image_col_data[coord.x][2] = class_color[2];
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Feature<float> CLanesPostprocessor::calculate_mean_feature_vector(const std::vector<DBSCANSample<float>>& input_samples)
{
	if (input_samples.empty())
	{
		return Feature<float>();
	}

	size_t feature_dims = input_samples[0].get_feature_vector().size();
	size_t sample_nums = input_samples.size();
	Feature<float> mean_feature_vec;
	mean_feature_vec.resize(feature_dims, 0.0);

	for (const auto& sample : input_samples)
	{
		for (size_t index = 0; index < feature_dims; ++index)
		{
			mean_feature_vec[index] += sample[index];
		}
	}
	for (size_t index = 0; index < feature_dims; ++index)
	{
		mean_feature_vec[index] /= sample_nums;
	}

	return mean_feature_vec;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Feature<float> CLanesPostprocessor::calculate_stddev_feature_vector(const std::vector<DBSCANSample<float>>& input_samples, const Feature<float>& mean_feature_vec)
{
	if (input_samples.empty())
	{
		return Feature<float>();
	}

	uint feature_dims = input_samples[0].get_feature_vector().size();
	uint sample_nums = input_samples.size();

	// calculate stddev feature vector
	Feature<float> stddev_feature_vec;
	stddev_feature_vec.resize(feature_dims, 0.0);

	for (const auto& sample : input_samples)
	{
		for (uint index = 0; index < feature_dims; ++index)
		{
			auto sample_feature = sample.get_feature_vector();
			auto diff = sample_feature[index] - mean_feature_vec[index];
			diff = std::pow(diff, 2);
			stddev_feature_vec[index] += diff;
		}
	}

	for (uint index = 0; index < feature_dims; ++index)
	{
		stddev_feature_vec[index] /= sample_nums;
		stddev_feature_vec[index] = std::sqrt(stddev_feature_vec[index]);
	}

	return stddev_feature_vec;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CLanesPostprocessor::normalize_sample_features(const std::vector<DBSCANSample<float>>& input_samples, std::vector<DBSCANSample<float>>& output_samples)
{
	// calculate mean feature vector
	Feature<float> mean_feature_vector = calculate_mean_feature_vector(input_samples);

	// calculate stddev feature vector
	Feature<float> stddev_feature_vector = calculate_stddev_feature_vector(input_samples, mean_feature_vector);
	std::vector<DBSCANSample<float>> input_samples_copy = input_samples;

	for (auto& sample : input_samples_copy)
	{
		auto feature = sample.get_feature_vector();

		for (unsigned long index = 0; index < feature.size(); ++index)
		{
			feature[index] = (feature[index] - mean_feature_vector[index]) / stddev_feature_vector[index];
		}

		sample.set_feature_vector(feature);
	}

	output_samples = input_samples_copy;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CLanesPostprocessor::simultaneously_random_shuffle(std::vector<cv::Point> src1, std::vector<DBSCANSample<float>> src2)
{

	if (src1.empty() || src2.empty())
	{
		return;
	}

	// construct index vector of two input src
	std::vector<uint> indexes;
	indexes.reserve(src1.size());
	std::iota(indexes.begin(), indexes.end(), 0);
	std::random_device rd;
	std::default_random_engine eng{ rd() };
	std::shuffle(indexes.begin(), indexes.end(), eng);

	// make copy of two input vector
	std::vector<cv::Point> src1_copy(src1);
	std::vector<DBSCANSample<float>> src2_copy(src2);

	// random two source input vector via random shuffled index vector
	for (unsigned long i = 0; i < indexes.size(); ++i)
	{
		src1[i] = src1_copy[indexes[i]];
		src2[i] = src2_copy[indexes[i]];
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CLanesPostprocessor::simultaneously_random_select(const std::vector<DBSCANSample<float>>& src1, const std::vector<cv::Point>& src2, float select_ratio, std::vector<DBSCANSample<float>>& output1, std::vector<cv::Point>& output2)
{
	// check if select ratio is right
	if (select_ratio < 0.0 || select_ratio > 1.0)
	{
		std::cout << "Select ratio should be in range [0.0, 1.0]";
		return;
	}

	// calculate selected element counts using ceil to get
	auto src_element_counts = src1.size();
	auto selected_elements_counts = static_cast<uint>(std::ceil(src_element_counts * select_ratio));

	// random shuffle indexes
	std::vector<uint> indexes = std::vector<uint>(src_element_counts);
	std::iota(indexes.begin(), indexes.end(), 0);
	std::random_device rd;
	std::default_random_engine eng{ rd() };
	std::shuffle(indexes.begin(), indexes.end(), eng);

	// select part of the elements via first selected_elements_counts index in random shuffled indexes vector
	output1.resize(selected_elements_counts);
	output2.resize(selected_elements_counts);

	for (uint i = 0; i < selected_elements_counts; ++i)
	{
		output1[i] = src1[indexes[i]];
		output2[i] = src2[indexes[i]];
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CLanesPostprocessor::set_cluster_parameters(CyC_UINT dbscan_core_object_min_pts = 100,
	float dbscan_neighbor_radius = 0.35f, float embedding_feats_dilution_ratio = 0.5f)
{
	_dbscan_core_object_min_pts = dbscan_core_object_min_pts;
	_dbscan_neighbor_radius = dbscan_neighbor_radius;
	_embedding_feats_dilution_ratio = embedding_feats_dilution_ratio;
}
