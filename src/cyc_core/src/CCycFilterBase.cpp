// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CCycFilterBase.h"
#include <opencv2/highgui.hpp>
#include <spdlog/sinks/basic_file_sink.h>

CCycFilterBase::CCycFilterBase(CycDatablockKey key) :
    m_FilterKey(key),
    m_FilterType(-1),
    m_bIsPublishable(false),
    m_bIsNetworkFilter(false),
    m_nReplayFilter(-1),
    m_dt(-1LL),
    m_dtSequencing(-1LL),
    m_OutputDataType(CyC_UNDEFINED),
    m_InputDataType(CyC_UNDEFINED),
    m_tSamplingTime(-1LL), // Initial sampling time is undefined (-1.)
    m_tTimestampStart(-1LL),
    m_tTimestampStop(-1LL),
    m_bIsEnabled(false),
    m_bIsRunning(false),
    m_bIsProcessing(false)
{
    // Use "." as decimal separator
    std::setlocale(LC_NUMERIC, "C");
}

CCycFilterBase::CCycFilterBase(const ConfigFilterParameters& params) :
    m_FilterKey(params.key),
    m_sFilterName(params.sName),
    m_FilterType(params.nFilterType),
    m_bIsPublishable(params.bIsPublishable),
    m_bIsNetworkFilter(params.bIsNetworkFilter),
    m_nReplayFilter(params.nReplayFromDB),
    m_dt(params.dt),
    m_dtSequencing(params.dtSequencing),
    m_sGlobalBasePath(params.sGlobalBasePath),
    m_CustomParameters(params.CustomParameters),
    m_OutputDataType(CyC_UNDEFINED),
    m_InputDataType(CyC_UNDEFINED),
    m_tSamplingTime(-1LL), // Initial sampling time is undefined (-1.)
    m_tTimestampStart(-1LL),
    m_tTimestampStop(-1LL),
    m_bIsEnabled(false),
    m_bIsRunning(false),
    m_bIsProcessing(false)
{
    // Use "." as decimal separator
    std::setlocale(LC_NUMERIC, "C");
    
    if (!spdlog::get("default_logger"))
    {
        spdlog::set_default_logger(
            spdlog::basic_logger_mt(
                "default_logger",
                params.sLogFile));
        spdlog::set_pattern("%v");
        spdlog::set_pattern("[%H:%M:%S %z] [%^%l%$] [thread %t] %v");
        spdlog::flush_every(std::chrono::seconds(1));
    }
}

CCycFilterBase::~CCycFilterBase()
{
    if (m_pSensorModel != nullptr)
        delete m_pSensorModel;
}

CycDatablockKey CCycFilterBase::getFilterKey()
{
    return m_FilterKey;
}

void CCycFilterBase::setFilterKey(const CycDatablockKey& f_key)
{
    m_FilterKey = f_key;
}

CyC_FILTER_TYPE CCycFilterBase::getFilterType()
{
    return m_FilterType;
}

void CCycFilterBase::setFilterType(const std::string& _filter_type)
{
    m_FilterType = CStringUtils::CyC_HashFunc(_filter_type);
}

std::string CCycFilterBase::getFilterName()
{
    return m_sFilterName;
}

bool CCycFilterBase::isPublishable()
{
    return m_bIsPublishable;
}

bool CCycFilterBase::isNetworkFilter()
{
    return m_bIsNetworkFilter;
}

void CCycFilterBase::setNetworkFilter(const bool& isNetworkFilter)
{
    m_bIsNetworkFilter = isNetworkFilter;
}

bool CCycFilterBase::isReplayFilter()
{
    if (m_nReplayFilter >= 0)
        return true;
    else
        return false;
}

CyC_INT CCycFilterBase::getReplayFilter()
{
    return m_nReplayFilter;
}

std::string CCycFilterBase::getGlobalBasePath()
{
    return m_sGlobalBasePath;
}

CBaseSensorModel* CCycFilterBase::getSensorModel()
{
    return m_pSensorModel;
}

CyC_TIME_UNIT CCycFilterBase::getDt()
{
    return m_dt;
}

CyC_TIME_UNIT CCycFilterBase::getDtSequencing()
{
    return m_dtSequencing;
}

CyC_TIME_UNIT CCycFilterBase::getSamplingTime()
{
    return m_tSamplingTime;
}

CyC_TIME_UNIT CCycFilterBase::getTimestampStart()
{
    return m_tTimestampStart;
}

CyC_TIME_UNIT CCycFilterBase::getTimestampStop()
{
    return m_tTimestampStop;
}

CyC_DATA_TYPE CCycFilterBase::getOutputDataType()
{
    return m_OutputDataType;
}

CyC_DATA_TYPE CCycFilterBase::getInputDataType()
{
    return m_InputDataType;
}

CycInputSources& CCycFilterBase::getInputSources()
{
    return m_InputSources;
}

bool CCycFilterBase::setInputSources(const CycInputSources& sources)
{
    m_InputSources.clear();
    std::copy(sources.begin(), sources.end(), std::back_inserter(m_InputSources));

    return true;
}

bool CCycFilterBase::isEnabled()
{
    return m_bIsEnabled;
}

bool CCycFilterBase::isRunning()
{
    return m_bIsRunning;
}

bool CCycFilterBase::isProcessing()
{
    return m_bIsProcessing;
}

bool CCycFilterBase::getInputSourcesEnabled()
{
    bool bReturn(true);

    for (const auto& inputSource : m_InputSources)
    {
        if (!inputSource.pCycFilter->isEnabled())
        {
            bReturn = false;
        }
    }

    return bReturn;
}

bool CCycFilterBase::getInputSourcesRunning()
{
    bool bReturn(true);

    for (const auto& inputSource : m_InputSources)
    {
        if (!inputSource.pCycFilter->isRunning())
        {
            bReturn = false;
        }
    }

    return bReturn;
}

bool CCycFilterBase::start()
{
    if (isEnabled())
    {
        m_bIsRunning = true;

        if (!isReplayFilter() && !m_bIsNetworkFilter)
        {
            // Start the acquisition thread
            m_ExecutionThread = std::thread(&CCycFilterBase::run, this);
        }
    }
    return true;
}

bool CCycFilterBase::stop()
{
    m_bIsRunning = false;

    // Stop the thread
    if (this->m_ExecutionThread.joinable())
        this->m_ExecutionThread.join();

    return true;
}

void CCycFilterBase::run()
{
    while (m_bIsRunning)
    {
        // Set the processing flag
        m_bIsProcessing = true;
        m_tTimestampStart = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        // Call te process function
        if (!process())
            m_tTimestampStop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        
        // Elapsed time
        CyC_TIME_UNIT elapsed_time = m_tTimestampStop - m_tTimestampStart;

        // Sleep acording to given dt
        CyC_TIME_UNIT sleep_time = m_dt - elapsed_time;
        if (sleep_time > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        else
            sleep_time = 0;

        // Update sampling time
        m_tSamplingTime = elapsed_time + sleep_time;

        // Unset the processing flag
        m_bIsProcessing = false;
        //std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}

std::unordered_map<CycDatablockKey, CyC_TIME_UNIT> CCycFilterBase::getSync(CyC_TIME_UNIT ts_query)
{
    std::unordered_map<CycDatablockKey, CyC_TIME_UNIT> ret;

    if (m_DataCache.size() > 0)
    {
        if (ts_query == CyC_TIME_UNIT(-1))
        {
            ts_query = *m_DataCache.keys().rbegin();
            ret = m_DataCache.sync(ts_query);
        }
        else if (m_DataCache.count(ts_query) == 0)
        {
            spdlog::warn("Filter [{}-{}] [{}]: ERROR: No data in cache at timestamp key {}", getFilterKey().nCoreID, getFilterKey().nFilterID, getFilterName(), ts_query);
        }
        else
        {
            ret = m_DataCache.sync(ts_query);
        }
    }

    return ret;
}
