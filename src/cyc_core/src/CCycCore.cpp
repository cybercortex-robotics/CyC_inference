// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <filesystem>
#include <opencv2/imgproc.hpp>
#include "CCycCore.h"
#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include "CConfigParameters.h"
#include "CConfigCheck.h"
#include "dynalo/dynalo.hpp"
#include "os/qtplot/qtplot.h"

CCycCore::CCycCore() :
    m_nCoreID(-1),
    m_bCoreRunning(true)
{}

CCycCore::~CCycCore()
{
    // force logger to flush
    spdlog::default_logger()->flush();
    m_bCoreRunning = false;
}

bool CCycCore::init(const std::string& _conf_file, const std::vector<std::string>& _network_cores_conf_files, const bool& _viz_enabled)
{
    const bool bInitFromConfigFile = CConfigParameters::instance().init(_conf_file, false);

    if (bInitFromConfigFile)
    {
        //check config file
        if(!CConfigCheck::check(_conf_file))
           return false;

        // Initialize QT
        if (_viz_enabled)
        {
            if (m_SingletonRegistry.get<CCycQTSkeleton>() == nullptr)
                m_SingletonRegistry.registerInstance<CCycQTSkeleton>(&m_SingletonRegistry);
        }

        // Load shared filters libraries
        loadDlls();
        
        // Init core
        m_nCoreID = CConfigParameters::instance().getCoreID();

        // Init default visualization filter
        m_nStartupFiltersView = CConfigParameters::instance().getStartupFiltersView();

        // Init replay database path
        m_sReplayDBPath = CConfigParameters::instance().getReplayDBPath();

        // Parse local filters in the main configuration file
        std::vector<ConfigFilterParameters> FiltersConfiguration;
        bool bReadFiltersConfiguration = CConfigParameters::instance().getFiltersConfigParameters(FiltersConfiguration);

        if (bReadFiltersConfiguration)
        {
            // Parse the filters
            for (CyC_UINT i = 0; i < FiltersConfiguration.size(); i++)
            {
                // Add filter to Datablock only if it is activated
                if (FiltersConfiguration[i].bIsActive)
                {
                    ConfigFilterParameters& config = FiltersConfiguration[i];
                    CCycFilterBase* pFilter = nullptr;

                    // Singletons register
                    config.pSingletonRegistry = &m_SingletonRegistry;
                    
                    // Allocate memory and register filter in Datablock
                    if (mallocFilter(pFilter, config))
                        if (pFilter != nullptr)
                            registerFilter(pFilter);
                }
            }

            // Parse network filters from additional given network configurations
            //std::cout << std::endl << "Network configuration files with publishable filters:" << std::endl;
            for (std::string net_conf_file : _network_cores_conf_files)
            {
                std::cout << net_conf_file << std::endl;

                // Assign unique filter ID based on the core ID
                //static CyC_UINT uniqueFilterID = getVisionCoreID() * 10U;

                // Parse local filters from the network files
                std::vector<ConfigFilterParameters> FiltersConfigurationNetwork;
                CConfigParameters local_config;
                const bool bInitFromConfigFileNetwork = local_config.init(net_conf_file, true);
                bool bReadFiltersConfiguration = local_config.getFiltersConfigParameters(FiltersConfigurationNetwork);

                for (CyC_UINT i = 0; i < FiltersConfigurationNetwork.size(); i++)
                {
                    // Add filter to Datablock only if it is activated
                    if (FiltersConfigurationNetwork[i].bIsActive)
                    {
                        const ConfigFilterParameters& configNetwork = FiltersConfigurationNetwork[i];
                        CCycFilterBase* pFilter = nullptr;

                        // Allocate memory
                        if (mallocFilter(pFilter, configNetwork))
                        {
                            if (pFilter != nullptr)
                            {
                                // Allocate the unique ID
                                // TODO: check if ok
                                pFilter->setFilterKey(configNetwork.key);

                                // Set as network filter
                                pFilter->setNetworkFilter(true);

                                // Register filter in Datablock
                                registerFilter(pFilter);

                                //Connect input sources based on the core ID and filter ID
                                CycInputSources sources;
                                sources.emplace_back();

                                // TODO: check if ok
                                sources.back().SourceKey = configNetwork.key;

                                pFilter->setInputSources(sources);
                            }
                        }
                    }
                }
                std::cout << std::endl;
            }

            // Connect filters
            for (CyC_UINT i = 0; i < FiltersConfiguration.size(); ++i)
            {
                const ConfigFilterParameters& config = FiltersConfiguration[i];

                CycDatablockEntry* entry = nullptr;
                if (m_Datablock.readEntry(config.key, entry))
                {
                    connectInputSources(config.InputSources, entry->pCycFilter);
                }
            }
        }
    }

    return bInitFromConfigFile;
}

bool CCycCore::loadDlls()
{
    fs::path dlls_folder_path = fs::path(CConfigParameters::instance().getBasePath()) / fs::path(CConfigParameters::instance().getFiltersPath());

    for (const auto& dll : fs::directory_iterator(dlls_folder_path))
    {
        std::string extension;
#ifdef _WIN32
        extension = ".dll";
#else
        extension = ".so";
#endif
        if (dll.path().extension().compare(extension) == 0)
        {
            try
            {
                fs::path dll_no_ext = dll.path().parent_path() / (dll.path().stem().string() + extension);
                dynalo::library lib(dll_no_ext);
                auto getFilterType = lib.get_function<CyC_FILTER_TYPE()>("getFilterType");
                CyC_FILTER_TYPE filter_type = getFilterType();
                m_SharedFilters[filter_type] = std::make_unique<dynalo::library>(dll_no_ext);
            }
            catch (const std::exception& e)
            {
                spdlog::error("CycCore: {}", e.what());
            }
        }
    }

    return true;
}

bool CCycCore::mallocFilter(CCycFilterBase*& pFilter, const ConfigFilterParameters& config)
{
    bool bReturn(true);

    if (m_SharedFilters.find(config.nFilterType) != m_SharedFilters.end())
    {
        pFilter = m_SharedFilters[config.nFilterType]->get_function<CCycFilterBase * (const ConfigFilterParameters)>("createFilter")(config);
        spdlog::info("Loaded filter '{}'", pFilter->getFilterName());
    }
    else
    {
        spdlog::error("Filter '{}' could not be loaded.", config.nFilterType);
    }

    return bReturn;
}

bool CCycCore::registerFilter(CCycFilterBase* pFilter)
{
    CycDatablockEntry* pDatablockEntry = initDatablockEntry(pFilter->getFilterKey());
    pDatablockEntry->pCycFilter = pFilter;

    // Insert filter into the Datablock
    m_Datablock.insertEntry(pDatablockEntry);

    return true;
}

bool CCycCore::connectInputSources(const CycDatablockKeys& sources, CCycFilterBase* pFilter)
{
    bool bSourceConnected(true);
    
    CycInputSources InputSources;
    for (CyC_UINT i = 0; i < sources.size(); i++)
    {
        CycInputSource source;
        source.SourceKey.nCoreID = sources[i].nCoreID;
        source.SourceKey.nFilterID = sources[i].nFilterID;
        source.sDescription = sources[i].sDescription;
        
        if (!pFilter->isNetworkFilter())
            bool bConnected = readFilter(source.SourceKey, source.pCycFilter);
        else
            source.pCycFilter = nullptr;
        
        InputSources.push_back(source);
    }

    if (bSourceConnected)
        pFilter->setInputSources(InputSources);
    
    return  bSourceConnected;
}

bool CCycCore::readFilter(CycDatablockKey key, CCycFilterBase*& pFilter)
{
    bool bReturn(false);

    CycDatablockEntry* pEntry;
    bReturn = m_Datablock.readEntry(key, pEntry);

    if (bReturn == true)
        pFilter = pEntry->pCycFilter;
    
    return bReturn;
}

bool CCycCore::deleteFilter(CycDatablockKey key)
{
    return m_Datablock.deleteEntry(key);
}

CycDatablockEntry* CCycCore::initDatablockEntry(CycDatablockKey key)
{
    CycDatablockEntry* pDatablockEntry = new CycDatablockEntry;
    pDatablockEntry->Key = key;
    pDatablockEntry->pCycFilter = NULL;

    return pDatablockEntry;
}

void CCycCore::printDatablock()
{
    // Print Datablock statistics
    std::cout << std::endl << "***** Datablock content statistics ***** " << std::endl <<
        "Local Core ID:\t" << this->getVisionCoreID() << std::endl <<
        "Registered filters:\t" << m_Datablock.getNumberOfEntries() << std::endl;

    // Print Datablock
    m_Datablock.printDatablock();
}

bool CCycCore::getCycImage2CvMat(CCycFilterBase* pFilter, cv::Mat& dst, const CyC_TIME_UNIT& ts_query)
{
    CycImages cycImgs;
    float fDepthScaleFactor = 100.f / 16.f;  // Factor used to convert from meters to depth in range of [0-255]: 1000.f / 16.f;
    const bool bDataRead = pFilter->getData(cycImgs, ts_query);

    if (bDataRead)
    {
        for (const CycImage_& cycImg : cycImgs)
        {
            if (cycImg.bIsStereo /*&& pFilter->getFilterType() != CyC_RGBDCAMERA_FILTER_TYPE*/)
            {
                cv::Mat strip;
                cv::Mat imgL(cycImg.nRows, cycImg.nCols, cycImg.nType1);
                imgL.data = static_cast<uchar*>(cycImg.pData1);

                cv::Mat imgR(cycImg.nRows, cycImg.nCols, cycImg.nType2);
                imgR.data = static_cast<uchar*>(cycImg.pData2);

                // stereo RGB image
                if (cycImg.nType1 == cycImg.nType2)
                {
                    cv::vconcat(imgL, imgR, strip);
                }
                // RGBD image
                else
                {
                    // convert depth from 16UC1 to 8UC3 to be able to concatenate
                    cv::Mat imgR8UC1, imgR8UC3;
                    imgR.convertTo(imgR8UC1, CV_8UC1, fDepthScaleFactor);
                    cv::cvtColor(imgR8UC1, imgR8UC3, cv::COLOR_GRAY2RGB);
                    cv::vconcat(imgL, imgR8UC3, strip);
                }

                if (dst.empty())
                    strip.copyTo(dst);
                else
                    hconcat(dst, strip, dst);
            }
            else
            {
                cv::Mat img(cycImg.nRows, cycImg.nCols, cycImg.nType1);
                img.data = static_cast<uchar*>(cycImg.pData1);

                if (dst.empty())
                    img.copyTo(dst);
                else
                    hconcat(dst, img, dst);
            }
        }
    }

    return bDataRead;
}
