// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CBaseCcrFilter_H_
#define CBaseCcrFilter_H_

#include "CCR_TYPES.h"
#include "sensors/CBaseSensorModel.h"
#include "core/CCcrCache.h"
#include "core/CConfigParameters.h"
#include <os/CTimer.h>

class CBaseCcrFilter;

/*
 * Structure mapping an input source key to a filter
 */
struct CcrInputSource
{
    CcrDatablockKey SourceKey;
    std::string     sDescription;
	CBaseCcrFilter* pCcrFilter = nullptr;

    template <typename DataType_t>
    bool getData(DataType_t& data, CCR_TIME_UNIT ts_query = -1);

private:
    CCR_TIME_UNIT tTimestampRead = -1;
};

typedef std::vector<CcrInputSource> CcrInputSources;

/*
 * Base filter class used to store and read data in the Datablock
 */
class CBaseCcrFilter
{
public:
    explicit CBaseCcrFilter(CcrDatablockKey key);
	explicit CBaseCcrFilter(const ConfigFilterParameters& params);
	virtual ~CBaseCcrFilter();

	CcrDatablockKey     getFilterKey();
    void                setFilterKey(const CcrDatablockKey& f_key);
	CCR_FILTER_TYPE     getFilterType();
	std::string         getFilterName();
	bool                isPublishable();
	bool                isNetworkFilter();
    void                setNetworkFilter(const bool& isNetworkFilter);
    bool                isReplayFilter();
    CCR_INT             getReplayFilter();
    std::string         getGlobalBasePath();

    CCR_TIME_UNIT       getDt();
    CCR_TIME_UNIT       getDtSequencing();
	CCR_TIME_UNIT       getSamplingTime();
    CCR_TIME_UNIT       getTimestampStart();
    CCR_TIME_UNIT       getTimestampStop();
	
	CCR_DATA_TYPE       getOutputDataType();
	CCR_DATA_TYPE       getInputDataType();
	bool                setInputSources(const CcrInputSources& sources);
	CcrInputSources&    getInputSources();

	virtual bool    enable() = 0;
	virtual bool    disable() = 0;
    virtual bool    start() final;
    virtual bool    stop() final;

    // Base sensor model, if the derived filter is a sensor
    CBaseSensorModel*   getSensorModel();

    template <typename DataType_t>
    void updateData(const DataType_t& data, 
        std::unordered_map<CcrDatablockKey, CCR_TIME_UNIT> sync = std::unordered_map<CcrDatablockKey, CCR_TIME_UNIT>(),
        CCR_TIME_UNIT ts_start = -1, CCR_TIME_UNIT ts_stop = -1, CCR_TIME_UNIT sampling_time = -1);

    template <typename DataType_t>
    bool getData(DataType_t& data, CCR_TIME_UNIT ts_query = -1);

    std::unordered_map<CcrDatablockKey, CCR_TIME_UNIT> getSync(CCR_TIME_UNIT ts_query = -1);

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

    static std::string get_log_format(CCR_ULONG coreID, CCR_INT filterID, const std::string& name, std::string format)
    {
        return fmt::format("Filter [{}-{}]: {}: {}", coreID, filterID, name, format);
    }

protected:
    CCR_DATA_TYPE       m_OutputDataType;
    CCR_DATA_TYPE       m_InputDataType;

    CCR_ATOMIC_BOOL     m_bIsEnabled;
    CCR_ATOMIC_BOOL     m_bIsProcessing;

    CBaseSensorModel*   m_pSensorModel = nullptr;

private:
    CCR_FILTER_TYPE         m_FilterType;
	CcrDatablockKey         m_FilterKey;
	std::string             m_sFilterName;
	bool                    m_bIsPublishable;
	bool                    m_bIsNetworkFilter;
    CCR_INT                 m_nReplayFilter;
    CCR_TIME_UNIT           m_dt;               // Desired sampling time, given in the configuration parameters
    CCR_TIME_UNIT           m_dtSequencing;     // Sequencing sampling time, given in the configuration parameters
    std::string             m_sGlobalBasePath;
    CcrInputSources         m_InputSources;

	CCR_ATOMIC_BOOL         m_bIsRunning;
    CCR_ATOMIC_TIME_UNIT    m_tTimestampStop;
    CCR_ATOMIC_TIME_UNIT    m_tTimestampStart;
    CCR_ATOMIC_TIME_UNIT    m_tSamplingTime;    // Actual sampling time

    std::mutex              m_Mutex;
    std::thread             m_ExecutionThread;
    CCcrCache               m_DataCache;
};

template <typename DataType_t>
bool CcrInputSource::getData(DataType_t& data, CCR_TIME_UNIT ts_query)
{
    bool result = false;
    if (pCcrFilter != nullptr)
    {
        const auto tsRead = pCcrFilter->getTimestampStop();
        if (tTimestampRead < tsRead)
        {
            result = pCcrFilter->getData(data, ts_query);
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
void CBaseCcrFilter::updateData(const DataType_t& data, 
    std::unordered_map<CcrDatablockKey, CCR_TIME_UNIT> sync,
    CCR_TIME_UNIT ts_start, CCR_TIME_UNIT ts_stop, CCR_TIME_UNIT sampling_time)
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
bool CBaseCcrFilter::getData(DataType_t& data, CCR_TIME_UNIT ts_query)
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

    if (ts_query == CCR_TIME_UNIT(-1))
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

#endif /* CBaseCcrFilter_H_ */
