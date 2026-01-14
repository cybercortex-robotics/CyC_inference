// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CDnnConfig.h"
#include <iostream>
#include <regex>
#include <os/CCsvReader.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <os/CFileUtils.h>

#ifdef __ANDROID_API__
bool g_loggerInitialized = false;
#endif


CDnnConfig::CDnnConfig(const std::string& _dnn_config_file)
{
    m_bIsInitialized = false;

	// Check if file exists
	bool bFileExists = CFileUtils::FileExist(_dnn_config_file.c_str());
	
	if (bFileExists)
	{
		libconfig::Config LibConfigFile;

		try
		{
			LibConfigFile.readFile(_dnn_config_file.c_str());
			const libconfig::Setting& rootConfig = LibConfigFile.getRoot();
			const libconfig::Setting& Common = rootConfig["Common"];

			Common.lookupValue("device", m_Common.device);
			Common.lookupValue("is_training", m_Common.is_training);
			Common.lookupValue("epochs", m_Common.epochs);
			Common.lookupValue("learning_rate", m_Common.learning_rate);
			Common.lookupValue("momentum", m_Common.momentum);
			Common.lookupValue("batch_size", m_Common.batch_size);
			Common.lookupValue("shuffle", m_Common.shuffle);
			Common.lookupValue("num_workers", m_Common.num_workers);
			Common.lookupValue("train_split", m_Common.train_split);
			Common.lookupValue("optimizer", m_Common.optimizer);
			Common.lookupValue("tensorboard", m_Common.tensorboard);
			Common.lookupValue("plot_architecture", m_Common.plot_architecture);
			Common.lookupValue("view_predictions", m_Common.view_predictions);
			Common.lookupValue("ckpts_dir", m_Common.ckpts_dir);
			Common.lookupValue("ckpt_freq", m_Common.ckpt_freq);
			Common.lookupValue("load_last_ckpt", m_Common.load_last_ckpt);
			Common.lookupValue("onnx_export", m_Common.onnx_export);
			Common.lookupValue("onnx_opset_version", m_Common.onnx_opset_version);
			Common.lookupValue("onnx_model_file", m_Common.onnx_model_file);
			Common.lookupValue("load_pretrained_weights", m_Common.load_pretrained_weights);
			Common.lookupValue("pretrained_weights", m_Common.pretrained_weights);
			
			// input_data
			libconfig::Setting& input_data = Common.lookup("input_data");
			for (CyC_INT i = 0; i < input_data.getLength(); ++i)
			{
				m_Common.input_data.emplace_back(CConversions::String2DataType(input_data[i].c_str()));
			}
			// input_shape
			libconfig::Setting& input_shape = Common.lookup("input_shape");
			for (CyC_INT i = 0; i < input_shape.getLength(); ++i)
			{
				std::vector<CyC_INT> shape;
				for (CyC_INT j = 0; j < input_shape[i].getLength(); ++j)
				{
					if (input_shape[i][j].isNumber())
						shape.emplace_back(input_shape[i][j]);
					else
						shape.emplace_back(-1);
				}
				m_Common.input_shape.emplace_back(shape);
			}

			// output_data
			libconfig::Setting& output_data = Common.lookup("output_data");
			for (CyC_INT i = 0; i < output_data.getLength(); ++i)
				m_Common.output_data.emplace_back(CConversions::String2DataType(output_data[i].c_str()));

			// output_shape
			libconfig::Setting& output_shape = Common.lookup("output_shape");
			for (CyC_INT i = 0; i < output_shape.getLength(); ++i)
			{
				std::vector<CyC_INT> shape;
				for (CyC_INT j = 0; j < output_shape[i].getLength(); ++j)
				{
					if (output_shape[i][j].isNumber())
						shape.emplace_back(output_shape[i][j]);
					else if (strcmp(output_shape[i][j].c_str(), "?") == 0)
							shape.emplace_back(-1);
				}
				m_Common.output_shape.emplace_back(shape);
			}

			// input_data_transforms
			libconfig::Setting& input_data_transforms = Common.lookup("input_data_transforms");
			for (CyC_INT i = 0; i < input_data_transforms.getLength(); ++i)
			{
				Transforms transforms;
				for (CyC_INT j = 0; j < input_data_transforms[i].getLength(); ++j)
				{
					Transform trans;
					trans.name = input_data_transforms[i][j][0].c_str();
					trans.type = String2TransformationType(trans.name);
					
					for (CyC_INT k = 1; k < input_data_transforms[i][j].getLength(); ++k)
					{
						Eigen::VectorXf params = Eigen::VectorXf::Zero(input_data_transforms[i][j][k].getLength());
						for (CyC_INT q = 0; q < input_data_transforms[i][j][k].getLength(); ++q)
						{
							params[q] = input_data_transforms[i][j][k][q];
						}
						trans.paramenters.emplace_back(params);
					}
					transforms.emplace_back(trans);
				}
				m_Common.input_data_transforms.emplace_back(transforms);
			}
		}
		catch (libconfig::ParseException& ex)
		{
			spdlog::error("Failed to read configuration with error: {} at line {}", ex.getError(), ex.getLine());
		}

		// Assertion checks
		// Check if the model file exists
		bool bModelFileExists = CFileUtils::FileExist(m_Common.onnx_model_file.c_str());
		if (!bModelFileExists)
			spdlog::error("CDnnConfig: onnx model file '{}' does not exist.", m_Common.onnx_model_file);

		// Check if the input data and transforms shapes are consistent
		bool bInputDataConsistent = false;
		if ((m_Common.input_data.size() == m_Common.input_shape.size()) && 
			(m_Common.input_data.size() == m_Common.input_data_transforms.size()))
			bInputDataConsistent = true;
		else
			spdlog::error("CDnnConfig: input data and transformation shapes are inconsistent.");

		// Check if the output data shapes are consistent
		bool bOutputDataConsistent = false;
		if (m_Common.output_data.size() == m_Common.output_shape.size())
			bOutputDataConsistent = true;
		else
			spdlog::error("CDnnConfig: output data shapes are inconsistent.");

		if (bModelFileExists && bInputDataConsistent && bOutputDataConsistent)
			m_bIsInitialized = true;
	}
}

CDnnConfig::~CDnnConfig()
{}

CDnnConfig::TransformType CDnnConfig::String2TransformationType(const std::string& sTransformationType)
{
	CDnnConfig::TransformType nTransformationType(CDnnConfig::Transform_UNDEFINED);

	if (strcmp(sTransformationType.c_str(), "RGB2BGR") == 0)
	{
		nTransformationType = CDnnConfig::Transform_RGB2BGR;
	}
	else if (strcmp(sTransformationType.c_str(), "BGR2RGB") == 0)
	{
		nTransformationType = CDnnConfig::Transform_RGB2BGR;
	}
	else if (strcmp(sTransformationType.c_str(), "RGB2GRAY") == 0)
	{
		nTransformationType = CDnnConfig::Transform_RGB2GRAY;
	}
	else if (strcmp(sTransformationType.c_str(), "BGR2GRAY") == 0)
	{
		nTransformationType = CDnnConfig::Transform_RGB2GRAY;
	}
	else if (strcmp(sTransformationType.c_str(), "GRAY2RGB") == 0)
	{
		nTransformationType = CDnnConfig::Transform_GRAY2RGB;
	}
	else if (strcmp(sTransformationType.c_str(), "Normalize") == 0)
	{
		nTransformationType = CDnnConfig::Transform_NORMALIZATION;
	}

	return nTransformationType;
}
