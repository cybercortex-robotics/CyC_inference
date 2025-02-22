// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CCONFIGPARAMETERS_H_
#define CCONFIGPARAMETERS_H_

#include <map>
#include "CCR_TYPES.h"
#include "os/CConversions.h"
#include "os/CInternetUtils.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
#include <algorithm>

using CustomParametersType = std::map<std::string, std::string>;

struct ConfigFilterParameters
{
    CcrDatablockKey         key;
    std::string			    sName;
    bool				    bIsActive;
    CCR_FILTER_TYPE		    nFilterType;
    bool				    bIsPublishable;
    bool                    bIsNetworkFilter;
    CCR_INT                 nReplayFromDB;
    CCR_TIME_UNIT           dt;                 // sampling time in milliseconds
    CCR_TIME_UNIT           dtSequencing;       // sequencing sampling time in milliseconds
    CcrDatablockKeys        InputSources;
    CustomParametersType    CustomParameters;
    std::string             sGlobalBasePath;
    std::string             sLogFile;

    ConfigFilterParameters() :
        key(-1, -1),
        bIsActive(false),
        nFilterType(-1),
        bIsPublishable(false),
        bIsNetworkFilter(false),
        nReplayFromDB(-1),
        dt(-1LL),
        dtSequencing(-1LL)
    {}
};

class CConfigParameters
{
public:
    CConfigParameters();
    CConfigParameters(const CConfigParameters&) = delete;
    CConfigParameters(CConfigParameters&&) = delete;
    CConfigParameters& operator=(const CConfigParameters&) = delete;
    CConfigParameters& operator=(CConfigParameters&&) = delete;
    ~CConfigParameters() = default;

    bool	init(const std::string& confFile, bool isNetworkConfig);
    bool	init_from_csv(const std::string& confFile);

    // Independent filters
    bool	getFiltersConfigParameters(std::vector<ConfigFilterParameters>& params);

    const CCR_ULONG             getCoreID()                 { return m_nCoreID; }
    const std::vector<CCR_INT>  getStartupFiltersView()     { return m_StartupFiltersView; };
    const std::string&          getReplayDBPath() const     { return m_sReplayDBPath; }
    const std::string&          getLogFile() const          { return m_sLogFile; }
    const std::string&          getBasePath() const         { return m_sGlobalBasePath; }
    const std::string&          getFiltersPath() const      { return m_sFiltersPath; }

    const ConfigFilterParameters& getFilterParameters(CcrDatablockKey key) const
    {
        auto it = std::find_if(m_FiltersConfiguration.begin(), m_FiltersConfiguration.end(),
            [key](const ConfigFilterParameters& config)
            {
                return (config.key.nCoreID == key.nCoreID) && (config.key.nFilterID == key.nFilterID);
            });

        if (it != m_FiltersConfiguration.end())
        {
            return *it;
        }

        spdlog::error("Failed to find configuration parameters for filter with ID {} - {}", key.nCoreID, key.nFilterID);

        static ConfigFilterParameters tmp;
        return tmp;
    }

    const ConfigFilterParameters& getFilterParameters(const std::string& filterName) const
    {
        auto it = std::find_if(m_FiltersConfiguration.begin(), m_FiltersConfiguration.end(),
            [&filterName](const ConfigFilterParameters& config) { return config.sName == filterName; });

        if (it != m_FiltersConfiguration.end())
        {
            return *it;
        }

        spdlog::error("Failed to find configuration parameters for filter with name {}", filterName);

        static ConfigFilterParameters tmp;
        return tmp;
    }

    static CConfigParameters& instance()
    {
        static CConfigParameters instance;
        return instance;
    }

private:
    bool readFiltersConfiguration(const libconfig::Setting& Filters, std::vector<ConfigFilterParameters>& FiltersConfiguration);

private:
    CCR_ULONG                           m_nCoreID;
    std::string                         m_Description;
    std::vector<CCR_INT>                m_StartupFiltersView;
    std::string                         m_sReplayDBPath;
    std::string                         m_sLogFile;
    bool                                m_bIsConfigInitialized;
    std::vector<ConfigFilterParameters> m_FiltersConfiguration;
    std::string                         m_confFile;
    std::string                         m_sGlobalBasePath = "";
    std::string				            m_sFiltersPath = "";
};

#endif /* CCONFIGPARAMETERS_H_ */
