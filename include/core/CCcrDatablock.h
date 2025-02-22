// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CCcrDatablock_H_
#define CCcrDatablock_H_

// #define ENABLE_CVPLOT
#ifdef WIN32
#define ENABLE_QTPLOT
#endif // WIN32

#include <deque>
#include <map>
#include "CCR_TYPES.h"
#include "CBaseCcrFilter.h"
#include "os/CConversions.h"
#include <queue>
#include <string>
#ifdef ENABLE_CVPLOT
#include "cvplot/cvplot.h"
#include "os/CComputeInspect.h"
#endif
#ifdef ENABLE_QTPLOT
#include <os/qtplot/qtplot.h>
#include "os/CComputeInspect.h"
#endif

/*
 * Datablock entry
 * Maps a DataBlock key to a Filter
 */
struct CcrDatablockEntry
{
    CcrDatablockKey Key;
    CBaseCcrFilter* pCcrFilter;
};
typedef std::vector<CcrDatablockEntry> CcrDatablockEntries;

struct CcrDatablockEntryInfo
{
    CcrDatablockEntryInfo(CcrDatablockKey	key, CCR_FILTER_TYPE filterType, CCR_DATA_TYPE dataType, bool enabled, bool running, double samplingTime)
    {
        this->Key = key;
        this->filterType = filterType;
        this->dataType = dataType;
        this->enabled = enabled;
        this->running = running;
        this->samplingTime = samplingTime;
    }
    CcrDatablockKey Key;
    CCR_FILTER_TYPE filterType;
    CCR_DATA_TYPE   dataType;
    bool        enabled;
    bool        running;
    double      samplingTime;
};
typedef std::vector<CcrDatablockEntryInfo> CcrDatablockEntriesInfo;

class CCcrDatablock
{
public:
	CCcrDatablock();
	~CCcrDatablock();

public:
	/*
	 * Insert entry function
	 * return: TRUE is insertion passed; FALSE if the entry key already exists
	 */
	bool insertEntry(CcrDatablockEntry* pEntry);

	/*
     * TBD - not yet implemented
	 * Update entry function
	 * return: TRUE is update passed; FALSE if the entry key does not exists
	 */
	bool updateEntry(CcrDatablockEntry* pEntry);

	/*
	 * Read entry function
	 * return: pointer to a CCR_Datablock_ENTRY variable
	 */
	bool readEntry(CcrDatablockKey key, CcrDatablockEntry*& pEntry);

	/*
	 * Delete entry function
	 * return: TRUE is deletion is succesfull; FALSE if the entry key does not exists
	 */
	bool deleteEntry(CcrDatablockKey key);

	/*
	 * Deletes all entries in the Datablock
	 * return: TRUE is deletion is succesfull; FALSE otherwise
	 */
	bool clearDatablock();

	/*
	 * Enables all the filters
	 */
	void enableAllFilters();

	/*
	 * Disables all the filters
	 */
	void disableAllFilters();

	/*
	 * Starts all the filters
	 */
	void startAllFilters();

	/*
	 * Stops all the filters
	 */
	void stopAllFilters();

	/*
	 * Content retrieval functions
	 */
	CCR_UINT getNumberOfEntries();

	/*
     * Returns the Datablock status and entries in a vector of entries
     */
    CcrDatablockEntriesInfo getDatablock() const;

    /*
     * Prints the Datablock status and entries
     */
    void printDatablock();
#if defined(ENABLE_CVPLOT) || defined(ENABLE_QTPLOT)
    /*
     * Start / Stops plotting the sampling time of the Datablock registered filters
     */
    void startDatablockPlot();
    void stopDatablockPlot();

private:
    /*
     * Initilizes the plotting object
     */
    void initPlotting();

    /*
     * Thread for saves the timestamps and sampling times in a FIFO structure
     */
    void cacheTimestampsThread();

    /*
     * Ploting thread
     */
    void plotDatablockThread();
#endif
private:
	/*
	 * Datablock entry key given by:
	 * 1. CCR Core ID
	 * 2. Filter ID
	 */
	using DatablockEntryKey = std::pair<CCR_UINT, CCR_UINT>;

	std::map<DatablockEntryKey, CcrDatablockEntry*>	m_DatablockMap;
	std::deque<CcrDatablockEntry*>                  m_DatablockDeque;

    // Plotting variables
#if defined(ENABLE_CVPLOT)
	CCcrPlot* m_pDatablockPlot = nullptr;
	CCcrPlot* m_pComputePlot = nullptr;
#endif
#if defined(ENABLE_QTPLOT)
	CCcrQTPlot* m_pDatablockPlotQT = nullptr;
	CCcrQTPlot* m_pComputePlotQT = nullptr;
#endif
#if defined(ENABLE_CVPLOT) || defined(ENABLE_QTPLOT)
	CComputeInspector					m_ComputeInspector;
    std::vector<std::string>			m_vDatablockSignalsNames;
	std::vector<std::string>			m_vComputeSignalsNames;
    CCR_ATOMIC_BOOL						m_bPlottingEnabled = false;
    CCR_ATOMIC_BOOL						m_bCachingThreadEnabled;
    std::thread							m_CachingThread;
    std::thread							m_PlottingThread;
    std::mutex							m_CachingMutex;
    std::vector<std::queue<CCR_INT>>	m_CachedDatablockSignals;
	std::vector<std::queue<float>>		m_CachedComputeSignals;
#endif
};

#endif /* CCcrDatablock_H_ */
