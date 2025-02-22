// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CCcrCore_H_
#define CCcrCore_H_

#include "CCcrDatablock.h"
#include "core/dynalo/dynalo.hpp"

class CCcrCore
{
public:
	CCcrCore();
	~CCcrCore();

	CCR_ULONG				getVisionCoreID() { return m_nCoreID; };
    std::string				getReplayDBPath() { return m_sReplayDBPath; };
	std::vector<CCR_INT>	getStartupFiltersView() { return m_nStartupFiltersView; };

	/**
	  * \brief Initialize the vision core
	  *
	  * \param conf_file					Main (local) configuration file of the vision core
	  * \param network_cores_conf_files     Vector of configuration files for generating network filters
	  **/
	bool init(const std::string& conf_file, const std::vector<std::string>& network_cores_conf_files);

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
	bool mallocFilter(CBaseCcrFilter*& pFilter, const ConfigFilterParameters& config);

	/*
	 * Insert a new filter in the Datablock
	 * return: True if insertion passed, False if the filter key already exists
	 */
	bool registerFilter(CBaseCcrFilter* pFilter);

	/*
	 * Connects a filter to its input sources
	 */
	bool connectInputSources(const CcrDatablockKeys& sources, CBaseCcrFilter* pFilter);

	/*
	 * Read filter from Datablock
	 */
	bool readFilter(CcrDatablockKey key, CBaseCcrFilter*& pFilter);

	/*
	 * Deletes a filter in the Datablock
	 * return: True if the deletion is succesfull, False otherwise
	 */
	bool deleteFilter(CcrDatablockKey key);

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
    CcrDatablockEntriesInfo getDatablock() { return m_Datablock.getDatablock(); };

    /*
     * Prints the Datablock status and entries
     */
    void printDatablock();

    /*
     * Visualization functions 
     */
    static bool getCcrImage2CvMat(CBaseCcrFilter* pFilter, cv::Mat& dst, const CCR_TIME_UNIT& ts = -1);

#ifdef __ANDROID_API__
    CCommunication* getCommunication() { return m_Communication; }
#endif

private:
	CcrDatablockEntry* initDatablockEntry(CcrDatablockKey key);

private:
	CCR_ULONG				m_nCoreID;
	bool					m_bCoreRunning;
	CCcrDatablock			m_Datablock;
	std::string				m_sReplayDBPath;
	std::vector<CCR_INT>	m_nStartupFiltersView;
	
	std::unordered_map<CCR_FILTER_TYPE, std::unique_ptr<dynalo::library>> m_SharedFilters;
};

#endif /* CCcrCore_H_ */
