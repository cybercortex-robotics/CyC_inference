// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CCycCore_H_
#define CCycCore_H_

#include "CCycDatablock.h"
#include "dynalo/dynalo.hpp"
#include "os/CSingletonRegistry.h"

class CCycCore
{
public:
	CCycCore();
	~CCycCore();

	CyC_ULONG				getVisionCoreID()		{ return m_nCoreID; };
    std::string				getReplayDBPath()		{ return m_sReplayDBPath; };
	std::vector<CyC_INT>	getStartupFiltersView() { return m_nStartupFiltersView; };
	CSingletonRegistry*		getSingletonRegistry()  { return &m_SingletonRegistry; };

	/**
	  * \brief Initialize the vision core
	  *
	  * \param _conf_file					Main (local) configuration file of the vision core
	  * \param _network_cores_conf_files    Vector of configuration files for generating network filters
	  * \param _viz_enabled					Initializes QT and stores its singleton in the singletons registers
	  **/
	bool init(const std::string& _conf_file, const std::vector<std::string>& _network_cores_conf_files, const bool& _viz_enabled = true);

	/**
	  * \brief Loads the available filter DLLs
	  **/
	bool loadDlls();

	/**
	  * \brief Allocates memory for a filter
	  *
	  * \param pFilter		Pointer to base filter for memory allocation
	  * \param config		Configuration parameters for the filter
	  * \param filter_type	Type of filter to instantiate
	  **/
	bool mallocFilter(CCycFilterBase*& pFilter, const ConfigFilterParameters& config);

	/*
	 * Insert a new filter in the Datablock
	 * return: True if insertion passed, False if the filter key already exists
	 */
	bool registerFilter(CCycFilterBase* pFilter);

	/*
	 * Connects a filter to its input sources
	 */
	bool connectInputSources(const CycDatablockKeys& sources, CCycFilterBase* pFilter);

	/*
	 * Read filter from Datablock
	 */
	bool readFilter(CycDatablockKey key, CCycFilterBase*& pFilter);

	/*
	 * Deletes a filter in the Datablock
	 * return: True if the deletion is succesfull, False otherwise
	 */
	bool deleteFilter(CycDatablockKey key);

	/*
	 * Deletes all entries in the Datablock
	 * return: TRUE is deletion is succesfull; FALSE otherwise
	 */
    bool clearDatablock() { return m_Datablock.clearDatablock(); };

	/*
	 * Enables all the filters
	 */
    void enableAllFilters() { m_Datablock.enableAllFilters(); };

	/*
	 * Disables all the filters
	 */
    void disableAllFilters() { m_Datablock.disableAllFilters(); };

	/*
	 * Starts all the filters
	 */
    void startAllFilters() { m_Datablock.startAllFilters(); };

	/*
	 * Stops all the filters
	 */
    void stopAllFilters() { m_Datablock.stopAllFilters(); };

    /*
     * Starts / Stops plotting the sampling time of the Datablock registered filters
     */
#if defined(ENABLE_CVPLOT) || defined(ENABLE_QTPLOT)
    void startDatablockPlot() { m_Datablock.startDatablockPlot(); };
    void stopDatablockPlot() { m_Datablock.stopDatablockPlot(); };
#else	
    void startDatablockPlot() {};
    void stopDatablockPlot() {};
#endif

    /*
     * Returns the Datablock status and entries
     */
    CycDatablockEntriesInfo getDatablock() { return m_Datablock.getDatablock(); };

    /*
     * Prints the Datablock status and entries
     */
    void printDatablock();

    /*
     * Visualization functions 
     */
    static bool getCycImage2CvMat(CCycFilterBase* pFilter, cv::Mat& dst, const CyC_TIME_UNIT& ts = -1);

#ifdef __ANDROID_API__
    CCommunication* getCommunication() { return m_Communication; }
#endif

private:
	CycDatablockEntry* initDatablockEntry(CycDatablockKey key);

private:
	CyC_ULONG				m_nCoreID;
	bool					m_bCoreRunning;
	CCycDatablock			m_Datablock;
	std::string				m_sReplayDBPath;
	std::vector<CyC_INT>	m_nStartupFiltersView;
	CSingletonRegistry		m_SingletonRegistry;

	std::unordered_map<CyC_FILTER_TYPE, std::unique_ptr<dynalo::library>> m_SharedFilters;
};

#endif /* CCycCore_H_ */
