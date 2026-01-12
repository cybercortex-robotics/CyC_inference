/************************************************
* Copyright 2019 Baidu Inc. All Rights Reserved.
* Author: MaybeShewill-CV
* File: lanenetModel.cpp
* Link: https://github.com/MaybeShewill-CV/lanenet-lane-detection/blob/master/mnn_project/lanenet_model.cpp
************************************************/
#pragma once
#include <CyC_TYPES.h>
//#include <omp.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <stdio.h>
#include <numeric>
#include <malloc.h>
#include <random>
#include <limits>
#include "os/CTimer.h"
#include "vision/clustering/dbscan.hpp"
#include "sensors/CPinholeCameraSensorModel.h"
#include "math/CPolynomialFitting.h"

class CLanesPostprocessor
{
private:
	//using DBSCANSample = DBSCANSample<float>;
	//using Feature = Feature<float>;

public:
	void cluster_lanes(const cv::Mat& tf_binary_seg_output, const cv::Mat& tf_instance_seg_output, 
		std::vector<cv::Point>& coords_ret, std::vector<std::vector<uint>>& clusters);

	void clusters2lanes(const std::vector<cv::Point>& coords_selected, const std::vector<std::vector<uint>>& cluster_ret, CycLanesModel& lanes_model);

	static void scale_lanes(const cv::Size& org_img_size, const cv::Size& inference_img_size, CycLanesModel& lanes_model);

	CycLane lane2world(const CycLane& lane_model, const cv::Point& org_img_size, const CPinholeCameraSensorModel& camera_model);

	void map_clusters_to_lanes_model(const std::vector<cv::Point>& coords_selected, 
		const std::vector<std::vector<uint>>& cluster_ret, const cv::Size& inference_img_size, CPinholeCameraSensorModel& camera_model, CycLanesModel& lanes_model);

	/***
	 * Gathet embedding features via binary segmentation mask
	 * @param binary_mask
	 * @param pixel_embedding
	 * @param coords
	 * @param embedding_features
	 */
	void gather_pixel_embedding_features(const cv::Mat& binary_mask, const cv::Mat& pixel_embedding,
		std::vector<cv::Point>& coords, std::vector<DBSCANSample<float>>& embedding_samples);

	/***
	 * Cluster pixel embedding features via DBSCAN
	 * @param embedding_samples
	 * @param cluster_ret
	 */
	void cluster_pixem_embedding_features(std::vector<DBSCANSample<float>> &embedding_samples,
		std::vector<std::vector<uint> > &cluster_ret, std::vector<uint>& noise);

	/***
	 * Visualize instance segmentation result
	 * @param cluster_ret
	 * @param coords
	 * @param instance_segmentation_result
	 */
	void visualize_instance_segmentation_result(const std::vector<std::vector<uint> >& cluster_ret,
		const std::vector<cv::Point>& coords, cv::Mat& instance_segmentation_result, float tensor_w, float tensor_h);

	/***
	 * Normalize input samples' feature. Each sample's feature is normalized via function as follows:
	 * feature[i] = (feature[i] - mean_feature_vector[i]) / stddev_feature_vector[i].
	 * @param input_samples : vector of samples whose feature vector need to be normalized
	 * @param output_samples : normalized result
	 */
	void normalize_sample_features(const std::vector<DBSCANSample<float>>& input_samples,
		std::vector<DBSCANSample<float>>& output_samples);

	/***
	 * Calculate the mean feature vector among a vector of DBSCAMSample samples
	 * @param input_samples : vector of DBSCAMSample samples
	 * @return : mean feature vector
	 */
	Feature<float> calculate_mean_feature_vector(const std::vector<DBSCANSample<float>>& input_samples);

	/***
	 * Calculate the stddev feature vector among a vector of DBSCAMSample samples
	 * @param input_samples : vector of DBSCAMSample samples
	 * @param mean_feature_vec : mean feature vector
	 * @return : stddev feature vector
	 */
	Feature<float> calculate_stddev_feature_vector(const std::vector<DBSCANSample<float>>& input_samples,
		const Feature<float>& mean_feature_vec);

	/***
	 * simultaneously random shuffle two vector inplace. The two input source vector should have the same size.
	 * @tparam T
	 * @param src1
	 * @param src2
	 */
	void simultaneously_random_shuffle(std::vector<cv::Point> src1, std::vector<DBSCANSample<float>> src2);

	/***
	 * simultaneously random select part of the two input vector into the two output vector.
	 * The two input source vector should have the same size because they have one-to-one mapping
	 * relation between the elements in two input vector
	 * @tparam T1 : type of input vector src1 which should support default constructor
	 *              due to the usage of vector resize function
	 * @tparam T2 : type of input vector src2 which should support default constructor
	 *              due to the usage of vector resize function
	 * @param src1 : input vector src1
	 * @param src2 : input vector src2
	 * @param select_ratio : select ratio which should within range [0.0, 1.0]
	 * @param output1 : selected partial vector of src1
	 * @param output2 : selected partial vector of src2
	 */
	void simultaneously_random_select(const std::vector<DBSCANSample<float>>& src1, const std::vector<cv::Point>& src2,
		float select_ratio, std::vector<DBSCANSample<float>>& output1, std::vector<cv::Point>& output2);

	void set_cluster_parameters(CyC_UINT dbscan_core_object_min_pts,
		float dbscan_neighbor_radius, float embedding_feats_dilution_ratio);

private:
	CyC_UINT _pix_embedding_feature_dims = 4;
	CyC_UINT _dbscan_core_object_min_pts = 75;
	float _dbscan_neighbor_radius = 0.4f;
	float _embedding_feats_dilution_ratio = 0.1f;
};
