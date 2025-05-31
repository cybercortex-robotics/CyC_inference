// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

// Used for tensorflow models

#ifndef CDNNCONFIGPARAMETERS_H_
#define CDNNCONFIGPARAMETERS_H_

#include <map>
#include "CyC_TYPES.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)

struct NeuralNetworkConfiguration
{
    std::string                 sNeuralNetworkPath;
    std::vector<std::string>    InputLayerNames;
    std::vector<CyC_INT>        InputLayerIndices;
    std::vector<std::string>    OutputLayerNames;
    std::vector<CyC_INT>        OutputLayerIndices;
};

class CDnnConfigParameters
{
public:
    CDnnConfigParameters(const CDnnConfigParameters&) = delete;
    CDnnConfigParameters(CDnnConfigParameters&&) = delete;
    CDnnConfigParameters& operator=(const CDnnConfigParameters&) = delete;
    CDnnConfigParameters& operator=(CDnnConfigParameters&&) = delete;
    ~CDnnConfigParameters() = default;

	bool init(const std::string& confFile);

    static const NeuralNetworkConfiguration getNeuralNetworkConfiguration(const std::string& confFile);

private:
    CDnnConfigParameters();

private:
    bool m_bIsConfigInitialized;
};

#endif /* CDNNCONFIGPARAMETERS_H_ */
