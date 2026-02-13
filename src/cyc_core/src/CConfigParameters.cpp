// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CConfigParameters.h"
#include <iostream>
#include <regex>
#include <os/CCsvReader.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <os/CFileUtils.h>
#include <os/CMacUtils.h>

#ifdef __ANDROID_API__
bool g_loggerInitialized = false;
#endif

CConfigParameters::CConfigParameters() :
    m_nCoreID(0),
    m_bIsConfigInitialized(false)
{}

bool CConfigParameters::init(const std::string& confFile, bool isNetworkConfig)
{
    m_StartupFiltersView.clear();

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

        // Core ID (either given or MAC address)
        if (rootConfig["Core"].exists("ID"))
        {
            if (rootConfig["Core"].lookup("ID").getType() == libconfig::Setting::TypeInt)
            {
                CyC_INT nTmp;
                rootConfig["Core"].lookupValue("ID", nTmp);
                m_nCoreID = static_cast<CyC_ULONG>(nTmp);
            }
            else if (rootConfig["Core"].lookup("ID").getType() == libconfig::Setting::TypeInt64)
            {
                m_nCoreID = rootConfig["Core"]["ID"];
            }
            else
            {
                std::cout << "CConfigParameters::init: Error: could not read Core ID from the configuration file." << std::endl;
            }
        }
        if (m_nCoreID <= 0)
            m_nCoreID = CMacUtils::getMAC();

        // Description of the core
        rootConfig["Core"].lookupValue("Description", m_Description);

        // Filters used for visualization at startup
        if (rootConfig["Core"].exists("StartupFiltersView"))
        {
            libconfig::Setting& views = rootConfig["Core"].lookup("StartupFiltersView");
            if (views.isNumber())
                m_StartupFiltersView.push_back(views);
            else if (views.isArray())
                for (CyC_INT i = 0; i < views.getLength(); ++i)
                    m_StartupFiltersView.push_back(views[i]);
        }
        
        // Replay DB path (if given)
        rootConfig["Core"].lookupValue("ReplayDB", m_sReplayDBPath);

        // Global base path
        if (rootConfig["Core"].exists("BasePath"))
            rootConfig["Core"].lookupValue("BasePath", m_sGlobalBasePath);

        // Location of the filters (DLLs)
        if (rootConfig["Core"].exists("Filters"))
            rootConfig["Core"].lookupValue("Filters", m_sFiltersPath);

        // Log file
        std::string log_file = "log.txt";
        if (rootConfig["Core"].exists("LogFile"))
        {
            rootConfig["Core"].lookupValue("LogFile", log_file);

            fs::path path = fs::path(m_sGlobalBasePath) / fs::path(log_file);

            //if (!fs::exists(path))
            //{
            //    std::cout << "Log file '" << log_file << "' does not exist. Exiting." << std::endl;
            //    exit(EXIT_FAILURE);
            //}
            //else
            {
                log_file = path;
            }
        }
        m_sLogFile = std::move(log_file);

        // Init logger
#ifdef __ANDROID_API__
        if (!g_loggerInitialized)
        {
            g_loggerInitialized = true;
#endif
            if (!isNetworkConfig)
            {
                if (!spdlog::get("default_logger"))
                {
                    spdlog::set_default_logger(
                        spdlog::basic_logger_mt(
                            "default_logger",
                            CConfigParameters::instance().getLogFile()));

                    spdlog::set_pattern("%v");
                    spdlog::info("\n== BEGIN SESSION ==");

                    spdlog::set_pattern("[%H:%M:%S %z] [%^%l%$] [thread %t] %v");
                    spdlog::flush_every(std::chrono::seconds(1));
                }
            }
#ifdef __ANDROID_API__
        }
#endif

        if (rootConfig.exists("Network"))
        {
            // Read the network configuration
            const libconfig::Setting& Network = rootConfig["Network"];

            bool networkActive = true;
            if (Network.exists("Active"))
            {
                Network.lookupValue("Active", networkActive);
            }

           /* m_localNetworkConfiguration.enabled = networkActive;
            if (networkActive)
            {
                if (Network.exists("SignalingPortPlain"))
                    Network.lookupValue("SignalingPortPlain", m_localNetworkConfiguration.signaling_port_plain);
                if (Network.exists("SignalingPortTls"))
                    Network.lookupValue("SignalingPortTls", m_localNetworkConfiguration.signaling_port_tls);
                if (Network.exists("SignalingUri"))
                    Network.lookupValue("SignalingUri", m_localNetworkConfiguration.signaling_uri);
            }*/
        }

        // Read the filters configuration
        const libconfig::Setting& Filters = rootConfig["Filters"];
        readFiltersConfiguration(Filters, m_FiltersConfiguration);

        m_bIsConfigInitialized = true;
    }

    return bFileExists;
}

bool CConfigParameters::readFiltersConfiguration(const libconfig::Setting& Filters,
    std::vector<ConfigFilterParameters>& FiltersConfiguration)
{
    CyC_UINT nCountFilters = Filters.getLength();

    // Clear the current configuration
    FiltersConfiguration.clear();

    // Read the new configuration parameters
    for (CyC_UINT i = 0; i < nCountFilters; i++)
    {
        ConfigFilterParameters configParams;
        configParams.sGlobalBasePath = m_sGlobalBasePath;
        configParams.sLogFile = m_sLogFile;

        const libconfig::Setting& FilterConfig = Filters[i];

        configParams.sName = Filters[i].getName();
        configParams.key.nCoreID = m_nCoreID;
        FilterConfig.lookupValue("ID", configParams.key.nFilterID);
        FilterConfig.lookupValue("Active", configParams.bIsActive);

        if (!configParams.bIsActive)
            continue;

        std::string sFilterType;
        FilterConfig.lookupValue("Type", sFilterType);
        configParams.nFilterType = CStringUtils::CyC_HashFunc(sFilterType);
        FilterConfig.lookupValue("IsPublishable", configParams.bIsPublishable);

        // Check if the filter is a replay filter
        if (FilterConfig.exists("ReplayFromDB"))
        {
            if (FilterConfig.lookup("ReplayFromDB").getType() == libconfig::Setting::TypeBoolean)
            {
                bool tmp;
                FilterConfig.lookupValue("ReplayFromDB", tmp);
                if (tmp)
                    configParams.nReplayFromDB = configParams.key.nFilterID;
                else
                    configParams.nReplayFromDB = -1;
            }
            else if (FilterConfig.lookup("ReplayFromDB").getType() == libconfig::Setting::TypeInt)
            {
                FilterConfig.lookupValue("ReplayFromDB", configParams.nReplayFromDB);
            }
        }

        float dt_sec, dt_sequencing_sec;
        FilterConfig.lookupValue("dt", dt_sec);
        FilterConfig.lookupValue("dt_Sequencing", dt_sequencing_sec);
        configParams.dt = static_cast<CyC_TIME_UNIT>(dt_sec * SEC2MSEC);
        configParams.dtSequencing = static_cast<CyC_TIME_UNIT>(dt_sequencing_sec * SEC2MSEC);

        if (FilterConfig.exists("IsNetworkFilter"))
            FilterConfig.lookupValue("IsNetworkFilter", configParams.bIsNetworkFilter);

        // Read the input sources
        const libconfig::Setting& InputSources = Filters[configParams.sName.c_str()]["InputSources"];

        for (CyC_INT j = 0; j < InputSources.getLength(); j++)
        {
            CycDatablockKey InputSourceParams;
            const libconfig::Setting& InputSourceConfig = InputSources[j];

            // Read source Core ID
            if (InputSourceConfig.exists("CoreID"))
            {
                if (InputSourceConfig.lookup("CoreID").getType() == libconfig::Setting::TypeInt)
                {
                    CyC_INT nTmp;
                    InputSourceConfig.lookupValue("CoreID", nTmp);
                    InputSourceParams.nCoreID = static_cast<CyC_ULONG>(nTmp);
                }
                else if (InputSourceConfig.lookup("CoreID").getType() == libconfig::Setting::TypeInt64)
                {
                    InputSourceParams.nCoreID = InputSourceConfig["CoreID"];
                }
                else
                {
                    std::cout << "CConfigParameters::readFiltersConfiguration: Error: could not read input source Core ID from the configuration file." << std::endl;
                }
            }
            else
            {
                InputSourceParams.nCoreID = m_nCoreID;
            }
            
            InputSourceConfig.lookupValue("FilterID", InputSourceParams.nFilterID);
            InputSourceConfig.lookupValue("Description", InputSourceParams.sDescription);

            configParams.InputSources.push_back(InputSourceParams);
        }

        // Read the filter parameters
        const libconfig::Setting& ExtraParameters = Filters[configParams.sName.c_str()]["Parameters"];

        for (CyC_INT j = 0; j < ExtraParameters.getLength(); j++)
        {
            std::string name, value;
            const libconfig::Setting& extra = ExtraParameters[j];

            extra.lookupValue("name", name);
            extra.lookupValue("value", value);

            configParams.CustomParameters[name] = value;
        }

        FiltersConfiguration.push_back(configParams);
    }

    return true;
}

bool CConfigParameters::init_from_csv(const std::string& confPath) 
{
    std::ifstream confFile(confPath);

    if (!confFile.is_open()) {
        m_bIsConfigInitialized = false;
        return false;
    }

    std::string line;
    std::getline(confFile, line); //header

    while (std::getline(confFile, line)) {

        ConfigFilterParameters configParams;
        std::string param;
        std::istringstream lstream(line);
        getline(lstream, param, ',');
        int coreID = stoi(param); // not used in ConfigParameters

        getline(lstream, param, ',');
        configParams.key.nFilterID = stoi(param);

        getline(lstream, param, ',');
        configParams.nFilterType = CStringUtils::CyC_HashFunc(param);


        std::regex paramsRegx("\\{.*?\\}");
        std::smatch paramMatch;
        getline(lstream, param, ',');
        if (!param.empty()) {
            while (std::regex_search(param, paramMatch, paramsRegx)) {
                std::string sourceParam = paramMatch.str();

                sourceParam.erase(sourceParam.begin());
                sourceParam.erase(sourceParam.end() - 1);

                CycDatablockKey InputSourceParams;
                std::istringstream lstream(sourceParam);
                std::regex numRegx("\\D");

                getline(lstream, sourceParam, ';');
                sourceParam = std::regex_replace(sourceParam, numRegx, "");
                InputSourceParams.nCoreID = std::stoi(sourceParam);

                getline(lstream, sourceParam, ';');
                sourceParam = std::regex_replace(sourceParam, numRegx, "");
                InputSourceParams.nFilterID = std::stoi(sourceParam);

                configParams.InputSources.push_back(InputSourceParams);
                param = paramMatch.suffix();
            }
        }

        getline(lstream, param, ','); //out date type, not used in ConfigParameters
        getline(lstream, param, ',');

        if (!param.empty()) {
            std::regex stringRegx("\".*?\"");
            std::smatch stringMatch;

            while (std::regex_search(param, paramMatch, paramsRegx)) {

                std::string extraParam = paramMatch.str();

                extraParam.erase(extraParam.begin());
                extraParam.erase(extraParam.end() - 1);

                std::regex_search(extraParam, stringMatch, stringRegx);
                std::string paramName = stringMatch.str();
                paramName.erase(paramName.begin());
                paramName.erase(paramName.end() - 1);

                extraParam = stringMatch.suffix();
                std::regex_search(extraParam, stringMatch, stringRegx);
                std::string paramVal = stringMatch.str();

                paramVal.erase(paramVal.begin());
                paramVal.erase(paramVal.end() - 1);

                configParams.CustomParameters[paramName] = paramVal;
                param = paramMatch.suffix();
            }
        }

        m_FiltersConfiguration.push_back(configParams);
    }

    m_bIsConfigInitialized = true;
    return true;
}

bool CConfigParameters::getFiltersConfigParameters(std::vector<ConfigFilterParameters>& params)
{
    bool bReturn(false);

    if (m_bIsConfigInitialized)
    {
        params = m_FiltersConfiguration;
        bReturn = true;
    }

    return bReturn;
}
