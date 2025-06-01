// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CCycFilterBase_H_
#define CCycFilterBase_H_

#include "CyC_TYPES.h"
#include "sensors/CBaseSensorModel.h"
#include "CCycCache.h"
#include "CConfigParameters.h"
#include <os/CTimer.h>

class CCycFilterBase;

/*
 * Structure mapping an input source key to a filter
 */
struct CycInputSource
{
    CycDatablockKey SourceKey;
    std::string     sDescription;
	CCycFilterBase* pCycFilter = nullptr;

    template <typename DataType_t>
    bool getData(DataType_t& data, CyC_TIME_UNIT ts_query = -1);

private:
    CyC_TIME_UNIT tTimestampRead = -1;
};

typedef std::vector<CycInputSource> CycInputSources;

/*
 * Base filter class used to store and read data in the Datablock
 */
class CCycFilterBase
{
public:
    explicit CCycFilterBase(CycDatablockKey key);
	explicit CCycFilterBase(const ConfigFilterParameters& params);
	virtual ~CCycFilterBase();

	CycDatablockKey     getFilterKey();
    void                setFilterKey(const CycDatablockKey& f_key);
	CyC_FILTER_TYPE     getFilterType();
	std::string         getFilterName();
	bool                isPublishable();
	bool                isNetworkFilter();
    void                setNetworkFilter(const bool& isNetworkFilter);
    bool                isReplayFilter();
    CyC_INT             getReplayFilter();
    std::string         getGlobalBasePath();

    CyC_TIME_UNIT       getDt();
    CyC_TIME_UNIT       getDtSequencing();
	CyC_TIME_UNIT       getSamplingTime();
    CyC_TIME_UNIT       getTimestampStart();
    CyC_TIME_UNIT       getTimestampStop();
	
	CyC_DATA_TYPE       getOutputDataType();
	CyC_DATA_TYPE       getInputDataType();
	bool                setInputSources(const CycInputSources& sources);
	CycInputSources&    getInputSources();

	virtual bool    enable() = 0;
	virtual bool    disable() = 0;
    virtual bool    start() final;
    virtual bool    stop() final;

    // Base sensor model, if the derived filter is a sensor
    CBaseSensorModel*   getSensorModel();

    template <typename DataType_t>
    void updateData(const DataType_t& data, 
        std::unordered_map<CycDatablockKey, CyC_TIME_UNIT> sync = std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>(),
        CyC_TIME_UNIT ts_start = -1, CyC_TIME_UNIT ts_stop = -1, CyC_TIME_UNIT sampling_time = -1);

    template <typename DataType_t>
    bool getData(DataType_t& data, CyC_TIME_UNIT ts_query = -1);

    std::unordered_map<CycDatablockKey, CyC_TIME_UNIT> getSync(CyC_TIME_UNIT ts_query = -1);

    // Must be implemented in a derived filter
    virtual void loadFromDatastream(const std::string& _datastream_entry, const std::string& _db_root_path) = 0;

	bool isEnabled();
	bool isRunning();
    bool isProcessing();

	bool getInputSourcesEnabled();
	bool getInputSourcesRunning();

    CustomParametersType m_CustomParameters;

protected:
    void setFilterType(const std::string& _filter_type);

    // Must be implemented in a derived filter
    virtual bool process() = 0;

    template <typename... Args>
    void log_info(std::string format, Args&&... args)
    {
        format = std::move(get_log_format(getFilterKey().nCoreID, getFilterKey().nFilterID, getFilterName(), std::move(format)));
        spdlog::info(format, args...);
    }
    
    template <typename... Args>
    void log_warn(std::string format, Args&&... args)
    {
        format = std::move(get_log_format(getFilterKey().nCoreID, getFilterKey().nFilterID, getFilterName(), std::move(format)));
        spdlog::warn(format, args...);
    }
    
    template <typename... Args>
    void log_error(std::string format, Args&&... args)
    {
        format = std::move(get_log_format(getFilterKey().nCoreID, getFilterKey().nFilterID, getFilterName(), std::move(format)));
        spdlog::error(format, args...);
    }

private:
    void run();

    static std::string get_log_format(CyC_ULONG coreID, CyC_INT filterID, const std::string& name, std::string format)
    {
        return fmt::format("Filter [{}-{}]: {}: {}", coreID, filterID, name, format);
    }

protected:
    CyC_DATA_TYPE       m_OutputDataType;
    CyC_DATA_TYPE       m_InputDataType;

    CyC_ATOMIC_BOOL     m_bIsEnabled;
    CyC_ATOMIC_BOOL     m_bIsProcessing;

    CBaseSensorModel*   m_pSensorModel = nullptr;

private:
    CyC_FILTER_TYPE                m_FilterType;
	CycDatablockKey         m_FilterKey;
	std::string             m_sFilterName;
	bool                    m_bIsPublishable;
	bool                    m_bIsNetworkFilter;
    CyC_INT                 m_nReplayFilter;
    CyC_TIME_UNIT           m_dt;               // Desired sampling time, given in the configuration parameters
    CyC_TIME_UNIT           m_dtSequencing;     // Sequencing sampling time, given in the configuration parameters
    std::string             m_sGlobalBasePath;
    CycInputSources         m_InputSources;

	CyC_ATOMIC_BOOL         m_bIsRunning;
    CyC_ATOMIC_TIME_UNIT    m_tTimestampStop;
    CyC_ATOMIC_TIME_UNIT    m_tTimestampStart;
    CyC_ATOMIC_TIME_UNIT    m_tSamplingTime;    // Actual sampling time

    std::mutex              m_Mutex;
    std::thread             m_ExecutionThread;
    CCycCache               m_DataCache;
};

template <typename DataType_t>
bool CycInputSource::getData(DataType_t& data, CyC_TIME_UNIT ts_query)
{
    bool result = false;
    if (pCycFilter != nullptr)
    {
        const auto tsRead = pCycFilter->getTimestampStop();
        if (tTimestampRead < tsRead)
        {
            result = pCycFilter->getData(data, ts_query);
            tTimestampRead = tsRead;
        }
    }

    return result;
}

template <typename ToDataType_t, typename FromDataType_t>
struct generic_copy
{
    static void copy(ToDataType_t& to, const FromDataType_t& from)
    {
        to = from;
    }
};

template <typename ToDataType_t>
struct generic_copy<ToDataType_t, CcrOcTree>
{
    static void copy(ToDataType_t& to, const CcrOcTree& from)
    {
        to.~CcrOcTree();
        new (&to) CcrOcTree(from.getResolution());

        // Information about color is lost when the copy constructor/operator is used
        // Maybe implement an efficient copy constructor for ColorOcTree?
        for (auto it = from.begin_leafs(); it != from.end_leafs(); ++it)
        {
            auto* node = to.updateNode(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z(), true, true);
            node->setColor(it->getColor());
        }
    }
};

template <typename DataType_t>
void CCycFilterBase::updateData(const DataType_t& data, 
    std::unordered_map<CycDatablockKey, CyC_TIME_UNIT> sync,
    CyC_TIME_UNIT ts_start, CyC_TIME_UNIT ts_stop, CyC_TIME_UNIT sampling_time)
{
    if (/*m_bIsRunning &&*/ m_bIsEnabled)
    {
        std::lock_guard<std::mutex> lock(m_Mutex);
        if (m_bIsNetworkFilter)
        {
            //ts_stop = CTimer::now();
            ts_stop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            m_DataCache.insert(ts_stop, data, sync);

            m_tTimestampStop = ts_stop;
            if (m_tTimestampStart != 0)
                m_tSamplingTime = m_tTimestampStop - m_tTimestampStart;
            
            //m_tTimestampStart = CTimer::now();
            m_tTimestampStart = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }
        else
        {
            if ((ts_start != -1) && (ts_stop != -1) && (sampling_time != -1))
            {
                m_DataCache.insert(ts_stop, data, sync);
                m_tTimestampStart = ts_start;
                m_tTimestampStop = ts_stop;
                m_tSamplingTime = sampling_time;
            }
            else if ((ts_start == -1) && (ts_stop == -1) && (sampling_time == -1))
            {
                //m_tTimestampStop = CTimer::now();
                ts_stop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                m_DataCache.insert(ts_stop, data, sync);

                m_tTimestampStop = ts_stop;
                //if (m_tTimestampStart != 0)
                //    m_tSamplingTime = m_tTimestampStop - m_tTimestampStart;
            }
            else
            {
                spdlog::error("Filter [{}]: updateData() ERROR: Timestamps inconsistent.", getFilterName());
            }
        }
    }
}

template <typename DataType_t>
bool CCycFilterBase::getData(DataType_t& data, CyC_TIME_UNIT ts_query)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    using value_t = typename std::decay<DataType_t>::type;

    if (m_DataCache.empty())
    {
        return false;
    }

    if (!m_DataCache.has_type(data))
    {
        spdlog::error("Filter [{}-{}] [{}]: ERROR: Wrong data type requested from cache.", getFilterKey().nCoreID, getFilterKey().nFilterID, getFilterName());
        return false;
    }

    if (ts_query == CyC_TIME_UNIT(-1))
    {
        ts_query = *m_DataCache.keys().rbegin();
    }

    if (m_DataCache.count(ts_query) == 0)
    {
        spdlog::warn("Filter [{}-{}] [{}]: ERROR: No data in cache at timestamp key {}", getFilterKey().nCoreID, getFilterKey().nFilterID, getFilterName(), ts_query);
        return false;
    }

    generic_copy<DataType_t, value_t>::copy(data, m_DataCache.at<value_t>(ts_query));
    return true;
}

#endif /* CCycFilterBase_H_ */
