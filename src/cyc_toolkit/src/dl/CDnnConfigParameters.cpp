// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

// Used for tensorflow models

#include "CDnnConfigParameters.h"
#include <iostream>
#include <regex>
#include <os/CCsvReader.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <os/CFileUtils.h>

#ifdef __ANDROID_API__
bool g_loggerInitialized = false;
#endif

CDnnConfigParameters::CDnnConfigParameters()
{}

bool CDnnConfigParameters::init(const std::string& confFile)
{
	// Check if file exists
	bool bFileExists = CFileUtils::FileExist(confFile.c_str());

	if (!bFileExists)
	{
		m_bIsConfigInitialized = false;
	}
	else
	{
		libconfig::Config configFile;
		try
		{
            configFile.readFile(confFile.c_str());
        }
		catch (libconfig::ParseException& ex)
        {
		    std::cout << "Failed to read configuration with error: " << std::endl;
		    std::cout << ex.getError() << " at line " << ex.getLine() << std::endl;
		    exit(-1);
        }

		const libconfig::Setting& rootConfig = configFile.getRoot();

		m_bIsConfigInitialized = true;
	}

	return m_bIsConfigInitialized;
}

const NeuralNetworkConfiguration CDnnConfigParameters::getNeuralNetworkConfiguration(const std::string& confFile)
{
    // Check if file exists
    bool bFileExists = CFileUtils::FileExist(confFile.c_str());
    NeuralNetworkConfiguration network_conf;

    if (bFileExists)
    {
        libconfig::Config configFile;
        configFile.readFile(confFile.c_str());

        const libconfig::Setting& rootConfig = configFile.getRoot();
        if (rootConfig.exists("ModelParameters"))
        {
            if (rootConfig["ModelParameters"].exists("ModelPath"))
            {
                rootConfig["ModelParameters"].lookupValue("ModelPath", network_conf.sNeuralNetworkPath);
            }
            else
            {
                spdlog::error("CDnnConfigParameters: ModelPath not found in neural network configuration file");
            }
            if (rootConfig["ModelParameters"].exists("InputLayerNames"))
            {
                for (const auto& layer : rootConfig["ModelParameters"]["InputLayerNames"])
                {
                    network_conf.InputLayerNames.push_back(layer);
                }
            }
            else
            {
                spdlog::error("CDnnConfigParameters: InputLayerNames not found in neural network configuration file");
            }
            if (rootConfig["ModelParameters"].exists("InputLayerIndex"))
            {
                for (const auto& idx : rootConfig["ModelParameters"]["InputLayerIndex"])
                {
                    network_conf.InputLayerIndices.push_back(idx);
                }
            }
            else
            {
                spdlog::error("CDnnConfigParameters: InputLayerIndex not found in neural network configuration file");
            }

            if (rootConfig["ModelParameters"].exists("OutputLayerNames"))
            {
                for (const auto& layer : rootConfig["ModelParameters"]["OutputLayerNames"])
                {
                    network_conf.OutputLayerNames.push_back(layer);
                }
            }
            else
            {
                spdlog::error("CDnnConfigParameters: OutputLayerNames not found in neural network configuration file");
            }
            if (rootConfig["ModelParameters"].exists("OutputLayerIndex"))
            {
                for (const auto& idx : rootConfig["ModelParameters"]["OutputLayerIndex"])
                {
                    network_conf.OutputLayerIndices.push_back(idx);
                }
            }
            else
            {
                spdlog::error("CDnnConfigParameters: InputLayerIndex not found in neural network configuration file");
            }
        }
        else
        {
            spdlog::error("CDnnConfigParameters: ModelParameters node not found in neural network configuration file");
        }
    }
    else
    {
        spdlog::error("CDnnConfigParameters: Configuration file {} not found. Model will be invalid", confFile);
    }

    return network_conf;
}
