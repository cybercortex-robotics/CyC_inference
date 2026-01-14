// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CCycDatablock.h"
#include "os/CCsvWriter.h"

CCycDatablock::CCycDatablock()
{}

CCycDatablock::~CCycDatablock()
{
	// Free Datablock memory
	clearDatablock();
    spdlog::info("Datablock cleared.");
}

bool CCycDatablock::insertEntry(CycDatablockEntry* pEntry)
{
	bool bReturn(true);

	DatablockEntryKey key{ pEntry->Key.nCoreID, pEntry->Key.nFilterID };
	m_DatablockMap[key] = pEntry;
    
	m_DatablockDeque.push_back(pEntry);
#if defined(ENABLE_QTPLOT) || defined(ENABLE_CVPLOT)
    // Init plotting functionality
    initPlotting();
#endif
	return bReturn;
}

bool CCycDatablock::updateEntry(CycDatablockEntry* pEntry)
{
	bool bReturn(true);
#if defined(ENABLE_QTPLOT) || defined(ENABLE_CVPLOT)
    // Init plotting functionality
    initPlotting();
#endif
	return bReturn;
}

bool CCycDatablock::readEntry(CycDatablockKey key, CycDatablockEntry*& pEntry)
{
	bool bReturn(false);
    
	DatablockEntryKey PairingKey{ key.nCoreID, key.nFilterID };

	if (m_DatablockMap.count(PairingKey))
	{
		pEntry = m_DatablockMap[PairingKey];
		bReturn = true;
	}

	return bReturn;
}

bool CCycDatablock::deleteEntry(CycDatablockKey key)
{
	bool bReturn(false);

	DatablockEntryKey PairingKey{ key.nCoreID, key.nFilterID };

	// Search the filter in the Datablock
	if (m_DatablockMap.count(PairingKey) != 0)
	{
		std::map<DatablockEntryKey, CycDatablockEntry*>::iterator itr = m_DatablockMap.find(PairingKey);

		// Stop the filter
		if (itr->second->pCycFilter->isRunning())
			itr->second->pCycFilter->stop();

		// Disable the filter
		if (itr->second->pCycFilter->isEnabled())
			itr->second->pCycFilter->disable();

		delete itr->second->pCycFilter;
		delete itr->second;

		m_DatablockMap.erase(itr);

		bReturn = true;
	}
#if defined(ENABLE_QTPLOT) || defined(ENABLE_CVPLOT)
    // Init plotting functionality
    initPlotting();
#endif
	return bReturn;
}

bool CCycDatablock::clearDatablock()
{
	bool bReturn(true);

	// Parse and delete all Datablock entries in reversed order of entry
	CyC_INT i = static_cast<CyC_INT>(m_DatablockDeque.size()) - 1;

	while (i >= 0)
	{
		// Stop the filter
		if (m_DatablockDeque[i]->pCycFilter->isRunning())
			m_DatablockDeque[i]->pCycFilter->stop();

		// Disable the filter
		if (m_DatablockDeque[i]->pCycFilter->isEnabled())
			m_DatablockDeque[i]->pCycFilter->disable();

		delete m_DatablockDeque[i]->pCycFilter;
		delete m_DatablockDeque[i];

		--i;
	}

	// Clear the Datablock deque
	m_DatablockDeque.clear();

    // Delete the plotting object
#ifdef ENABLE_CVPLOT
    delete m_pDatablockPlot;
    delete m_pComputePlot;
#endif
#ifdef ENABLE_QTPLOT
    if (m_pDatablockPlotQT)
    {
        m_pDatablockPlotQT->stop();
        delete m_pDatablockPlotQT;
        m_pDatablockPlotQT = nullptr;
    }

    if (m_pComputePlotQT)
    {
        m_pComputePlotQT->stop();
        delete m_pComputePlotQT;
        m_pComputePlotQT = nullptr;
    }
#endif

	return bReturn;
}

void CCycDatablock::enableAllFilters()
{
    for (size_t i = 0; i < m_DatablockDeque.size(); i++)
	{
		if (m_DatablockDeque[i]->pCycFilter->isEnabled() == false)
		{
			m_DatablockDeque[i]->pCycFilter->enable();
		}
	}
}

void CCycDatablock::disableAllFilters()
{
	for (size_t i = 0; i < m_DatablockDeque.size(); i++)
	{
		// Disable the filter
		if (m_DatablockDeque[i]->pCycFilter->isEnabled())
		{
			// Stop the filter first
			if (m_DatablockDeque[i]->pCycFilter->isRunning())
			{
				m_DatablockDeque[i]->pCycFilter->stop();
			}

			if(m_DatablockDeque[i]->pCycFilter->disable())
                spdlog::info("Filter [{}-{}] {} disabled successfully", 
                    m_DatablockDeque[i]->pCycFilter->getFilterKey().nCoreID,
                    m_DatablockDeque[i]->pCycFilter->getFilterKey().nFilterID, 
                    m_DatablockDeque[i]->pCycFilter->getFilterName());
		}
	}
}

void CCycDatablock::startAllFilters()
{
    for (size_t i = 0; i < m_DatablockDeque.size(); i++)
	{
		if (m_DatablockDeque[i]->pCycFilter->isRunning() == false)
		{
			if (m_DatablockDeque[i]->pCycFilter->isEnabled() == false)
			{
                m_DatablockDeque[i]->pCycFilter->enable();
			}
            
			m_DatablockDeque[i]->pCycFilter->start();
		}
	}
}

void CCycDatablock::stopAllFilters()
{
	for (size_t i = 0; i < m_DatablockDeque.size(); i++)
	{
		// Stop the filter
		if (m_DatablockDeque[i]->pCycFilter->isRunning())
		{
			m_DatablockDeque[i]->pCycFilter->stop();
		}
	}
}

CyC_UINT CCycDatablock::getNumberOfEntries()
{
	return static_cast<CyC_UINT>(m_DatablockDeque.size());
}

CycDatablockEntriesInfo CCycDatablock::getDatablock() const
{
    CycDatablockEntriesInfo Datablock;

    // Print filters status
    for (CyC_UINT i = 0; i < m_DatablockDeque.size(); i++)
    {
        bool enabled = m_DatablockDeque[i]->pCycFilter->isEnabled();
        bool running = m_DatablockDeque[i]->pCycFilter->isRunning();
        CycDatablockKey filterKey = m_DatablockDeque[i]->Key;
        CyC_UINT filterID = m_DatablockDeque[i]->Key.nFilterID;
        CyC_FILTER_TYPE filterType = m_DatablockDeque[i]->pCycFilter->getFilterType();
        CyC_DATA_TYPE outType = m_DatablockDeque[i]->pCycFilter->getOutputDataType();
        CyC_TIME_UNIT samplingTime = m_DatablockDeque[i]->pCycFilter->getSamplingTime();

        Datablock.emplace_back(filterKey, filterType, outType, enabled, running, samplingTime);
    }

    return Datablock;
}

void CCycDatablock::printDatablock()
{
    char cIsEnabled[8] = "False";
    char cIsRunning[8] = "False";

    std::cout << std::endl;

    // Defined table-head
    std::cout <<
        "CoreID\t" <<
        "FilterID\t" <<
        //"Type\t\t" <<
        //"Input\t" <<
        "Output\t" <<
        "Enabled\t" <<
        "Running\t" <<
        "Sampling time\t" <<
        "Name" << std::endl;

    // Print filters status
    for (CyC_UINT i = 0; i < m_DatablockDeque.size(); i++)
    {
        // Check each filter's status
        if (m_DatablockDeque[i]->pCycFilter->isEnabled())
            snprintf(cIsEnabled, sizeof(cIsEnabled), "True");
        else
            snprintf(cIsEnabled, sizeof(cIsEnabled), "False");

        if (m_DatablockDeque[i]->pCycFilter->isRunning())
            snprintf(cIsRunning, sizeof(cIsRunning), "True");
        else
            snprintf(cIsRunning, sizeof(cIsRunning), "False");

        std::string strSamplingTime;
        float fSamplingTime = (float)(m_DatablockDeque[i]->pCycFilter->getSamplingTime() * MSEC2SEC);

        // Prevent big numbers printing from initialization phase
        if (fSamplingTime > 999.F)
        {
            fSamplingTime = -1.F;
        }
        if (fSamplingTime < 0.f)
        {
            strSamplingTime = "N/A";
        }
        else
        {
            CyC_INT nr_of_decimals = 3;
            strSamplingTime = std::to_string(fSamplingTime).substr(0, std::to_string(fSamplingTime).find(".") + nr_of_decimals + 1);
        }
        
        CyC_ULONG core_id = m_DatablockDeque[i]->pCycFilter->getFilterKey().nCoreID;
        CyC_INT filter_id = m_DatablockDeque[i]->pCycFilter->getFilterKey().nFilterID;
        if (m_DatablockDeque[i]->pCycFilter->isNetworkFilter())
        {
            if (m_DatablockDeque[i]->pCycFilter->getInputSources().size() > 0)
            {
                core_id = m_DatablockDeque[i]->pCycFilter->getInputSources()[0].SourceKey.nCoreID;
                filter_id = m_DatablockDeque[i]->pCycFilter->getInputSources()[0].SourceKey.nFilterID;
            }
            else
            {
                core_id = -1;
                filter_id = -1;
            }
        }

        std::cout <<
            core_id << "\t" <<
            filter_id << "\t\t" <<
            //CConversions::FilterType2String(m_DatablockDeque[i]->pCycFilter->getFilterType()) << "\t" <<
            CConversions::DataType2String(m_DatablockDeque[i]->pCycFilter->getOutputDataType()) << "\t" <<
            cIsEnabled << "\t" <<
            cIsRunning << "\t" <<
            strSamplingTime << "\t\t" <<
            m_DatablockDeque[i]->pCycFilter->getFilterName() << std::endl;
    }
}

#if defined(ENABLE_CVPLOT) || defined(ENABLE_QTPLOT)
void CCycDatablock::startDatablockPlot()
{
    // Start the caching thread
    if (m_bCachingThreadEnabled == false)
    {
        m_bCachingThreadEnabled = true;
        m_CachingThread = std::thread(&CCycDatablock::cacheTimestampsThread, this);
    }
    
    // Start the plotting thread
    if (m_bPlottingEnabled == false)
    {
        m_bPlottingEnabled = true;
        m_PlottingThread = std::thread(&CCycDatablock::plotDatablockThread, this);
    }
    else
    {
        spdlog::error("CCycDatablock::startDatablockPlot(): plotting thread already started.");
    }
}

void CCycDatablock::stopDatablockPlot()
{
    // Stop the plotting thread
    m_bPlottingEnabled = false;
    if (m_PlottingThread.joinable())
        m_PlottingThread.join();

    // Stop the caching thread
    m_bCachingThreadEnabled = false;
    if (m_CachingThread.joinable())
        m_CachingThread.join();
}

void CCycDatablock::initPlotting()
{
    // Stop the plotting thread
    m_bPlottingEnabled = false;
    if (m_PlottingThread.joinable())
        m_PlottingThread.join();

    // Stop the caching thread
    m_bCachingThreadEnabled = false;
    if (m_CachingThread.joinable())
        m_CachingThread.join();

#ifdef ENABLE_CVPLOT
    delete m_pDatablockPlot;
    delete m_pComputePlot;
#endif

    // Clear the signals names
    m_vDatablockSignalsNames.clear();
    m_vComputeSignalsNames.clear();

    // Clear the cached timestamps and compute signals
    {
        std::lock_guard<std::mutex> lock(m_CachingMutex);

        for (auto& cache : m_CachedDatablockSignals)
        {
            std::queue<CyC_INT> q;
            std::swap(cache, q);
        }
        m_CachedDatablockSignals.clear();

        for (auto& cache : m_CachedComputeSignals)
        {
            std::queue<float> q;
            std::swap(cache, q);
        }
        m_CachedComputeSignals.clear();
    }

    // Parse the registered filters
    for (CyC_UINT i = 0; i < m_DatablockDeque.size(); ++i)
    {
        const auto& pEntry = m_DatablockDeque[i];
        m_vDatablockSignalsNames.push_back(pEntry->pCycFilter->getFilterName());

        {
            std::lock_guard<std::mutex> lock(m_CachingMutex);
            m_CachedDatablockSignals.push_back(std::queue<CyC_INT>()); // Init the signals caching FIFO queue
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }

    //  Init the signals caching FIFO queue
    m_CachedComputeSignals.push_back(std::queue<float>());
    m_CachedComputeSignals.push_back(std::queue<float>());
    m_CachedComputeSignals.push_back(std::queue<float>());
    m_CachedComputeSignals.push_back(std::queue<float>());
    m_vComputeSignalsNames.push_back("Total CPU usage");
    m_vComputeSignalsNames.push_back("CyberCortex.AI CPU share");
    m_vComputeSignalsNames.push_back("Total memory usage");
    m_vComputeSignalsNames.push_back("CyberCortex.AI memory share");

    std::vector<std::string> gpu_names;
    std::vector<CyC_ULONG> total_mem, free_mem, used_mem;
    if (m_ComputeInspector.getGpuInfo(gpu_names, total_mem, free_mem, used_mem))
    {
        for (const auto& gpu_name : gpu_names)
        {
            m_CachedComputeSignals.push_back(std::queue<float>());
            m_CachedComputeSignals.push_back(std::queue<float>());
            m_vComputeSignalsNames.push_back(gpu_name + " GPU usage");
            m_vComputeSignalsNames.push_back(gpu_name + " GPU memory usage");
        }
    }

    // Init the Datablock plot
#ifdef ENABLE_QTPLOT
    /*m_pDatablockPlotQT = new CCcrQTPlot("DataBlock signals");
    for (const std::string& s : m_vDatablockSignalsNames)
        m_pDatablockPlotQT->add_segment(s, {});

    m_pComputePlotQT = new CCcrQTPlot("Computational load");
    for (const std::string& s : m_vComputeSignalsNames)
        m_pComputePlotQT->add_segment(s, {});*/
#endif
#ifdef ENABLE_CVPLOT
    cv::Size datablock_plot_size(650, 500);
    cv::Size compute_plot_size(500, 500);
    m_pDatablockPlot = new CCycPlot("Datablock signals", datablock_plot_size, m_vDatablockSignalsNames);
    m_pComputePlot = new CCycPlot("Computational load", compute_plot_size, m_vComputeSignalsNames);
    // This is here because when instantiating CCycPlot the view buffer doesn't seem to get allocated...
    // Trying to get the buffer, will call ensure() which ensures the buffer is allocated.
    cvplot::Rect rect_datablock(0, 0, datablock_plot_size.width, datablock_plot_size.height);
    cvplot::Rect rect_compute(0, 0, compute_plot_size.width, compute_plot_size.height);
    (void)m_pDatablockPlot->m_View.buffer(rect_datablock);
    (void)m_pComputePlot->m_View.buffer(rect_compute);
#endif
}

void CCycDatablock::cacheTimestampsThread()
{
    CTimer timerBlock;

    while (m_bCachingThreadEnabled)
    {
        if (m_DatablockDeque.size() == m_CachedDatablockSignals.size())
        {
            timerBlock.restart();

            for (CyC_UINT i = 0; i < m_DatablockDeque.size(); ++i)
            {
                const auto& pEntry = m_DatablockDeque[i];

                {
                    std::lock_guard<std::mutex> lock(m_CachingMutex);

                    if (pEntry->pCycFilter->isProcessing())
                        m_CachedDatablockSignals[i].push(5);
                    else
                        m_CachedDatablockSignals[i].push(0);
                }
            }

            timerBlock.stop();

            // Sleep for 0.1ms (plotting sampling time = 0.1s)
            std::this_thread::sleep_for(std::chrono::microseconds(50 - timerBlock.getElapsedTimeMicroseconds()));
        }
    }
}

void CCycDatablock::plotDatablockThread()
{
#ifdef ENABLE_QTPLOT
    m_pDatablockPlotQT->run();
    m_pComputePlotQT->run();
#endif

    float time_scale = 0.4f;
    CyC_INT nr_of_fps = 50;
    std::vector<float> datablock_signals;
    std::vector<float> compute_signals;

    /*std::ofstream csv_writter("C:/data/computational_load.csv");
    csv_writter << "cpu_total,cpu_used_by_cybercortex,total_mem_used,total_mem_by_cybercortex,gpu_compute,gpu_memory" << std::endl;
    csv_writter.flush();*/

    while (m_bPlottingEnabled)
    {
        {
            std::lock_guard<std::mutex> lock(m_CachingMutex);

            // Update datablock signals
            datablock_signals.clear();
            for (CyC_UINT i = 0; i < m_CachedDatablockSignals.size(); ++i)
            {
                if (!m_CachedDatablockSignals[i].empty())
                {
                    float signal = (float)m_CachedDatablockSignals[i].front(); // 10.f + ((float)m_CachedDatablockSignals[i].front() - (float)i * 5.f);
                    datablock_signals.push_back(signal);
                    m_CachedDatablockSignals[i].pop();
                }
                else
                {
                    datablock_signals.push_back(0.f);
                }

                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }

            // CPU
            float cpu_total, cpu_used_by_process;
            m_ComputeInspector.getCpuUtilization(cpu_total, cpu_used_by_process);

            // GPU
            std::vector<float> gpus_compute, gpus_memory;
            m_ComputeInspector.getGpuUtilization(gpus_compute, gpus_memory);

            // Memory
            CyC_ULONG virtual_mem_total, virtual_mem_total_used, virtual_mem_used_by_current_process, physical_mem_total, physical_mem_total_used, physical_mem_used_by_current_process;
            m_ComputeInspector.getMemUtilization(virtual_mem_total, virtual_mem_total_used, virtual_mem_used_by_current_process, physical_mem_total, physical_mem_total_used, physical_mem_used_by_current_process);
            float virtual_mem_total_used_precentage = (static_cast<float>(virtual_mem_total_used) / static_cast<float>(virtual_mem_total)) * 100.f;
            float virtual_mem_used_by_current_process_precentage = (static_cast<float>(virtual_mem_used_by_current_process) / static_cast<float>(virtual_mem_total)) * 100.f;

            // Update compute signals
            m_CachedComputeSignals[0].push(cpu_total);
            m_CachedComputeSignals[1].push(cpu_used_by_process);
            m_CachedComputeSignals[2].push(virtual_mem_total_used_precentage);
            m_CachedComputeSignals[3].push(virtual_mem_used_by_current_process_precentage);

            CyC_INT idx = 4;
            for (size_t i = 0; i < gpus_compute.size(); ++i)
            {
                m_CachedComputeSignals[idx].push(gpus_compute[i]);
                ++idx;
                m_CachedComputeSignals[idx].push(gpus_memory[i]);
                ++idx;
            }

            compute_signals.clear();
            for (CyC_UINT i = 0; i < m_CachedComputeSignals.size(); ++i)
            {
                if (!m_CachedComputeSignals[i].empty())
                {
                    float signal =(float)m_CachedComputeSignals[i].front();
                    compute_signals.push_back(signal);
                    m_CachedComputeSignals[i].pop();
                }
                else
                {
                    compute_signals.push_back(0);
                }

                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }

            // Save computational load to CSV
            /*csv_writter << cpu_total << "," << cpu_used_by_process << "," << virtual_mem_total_used_precentage << "," << 
                virtual_mem_used_by_current_process_precentage << "," << gpus_compute[0] << "," << gpus_memory[0] << std::endl;*/
        }

#ifdef ENABLE_QTPLOT
        m_pDatablockPlotQT->plot_signals(datablock_signals);
        m_pComputePlotQT->plot_signals(compute_signals);
#endif
#ifdef ENABLE_CVPLOT
        cv::Mat disp_datablock, disp_compute, disp;
        m_pDatablockPlot->plot(datablock_signals, time_scale, nr_of_fps, disp_datablock);
        m_pComputePlot->plot(compute_signals, time_scale, nr_of_fps, disp_compute);
        cv::hconcat(disp_datablock, disp_compute, disp);
        cv::imshow(m_pDatablockPlot->m_Title, disp);
        cv::waitKey(1);
#endif
    }
}
#endif
