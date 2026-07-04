// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <queue>
#include <filesystem>
#include "CCycFilterBase.h"
#include "CConfigParameters.h"
#include "os/CConversions.h"
#include "CCycDatablock.h"
#include "CCycCore.h"
#include "CHddStorage.h"
#include "vision/CImageDisplayUtils.h"
#include "os/console_helper.hpp"
#include "os/CFileUtils.h"
#include "os/CCsvReader.h"
#include "CPlotFilterOutput.h"
#include "os/CComputeInspect.h"
#include "os/qtplot/qtimage.h"
#include "os/qtplot/qtplot.h"
#include "os/CSingletonRegistry.h"

#include <csignal>

#ifdef WIN32
// for timeBeginPeriod
#include <timeapi.h>
#endif // WIN32

#undef min

struct image_queue_vtype
{
    CyC_UINT filterID;
    cv::Mat img;
};

using image_queue_type = std::queue<image_queue_vtype>;

std::unordered_map<CyC_INT, bool> visualization_running_map;
std::unordered_map<CyC_INT, std::thread> visualization_thread_map;
std::unordered_map<CyC_UINT, std::string> window_names_map;

image_queue_type g_image_queue;
std::mutex g_iq_mutex;

CCycFilterBase* processCommand(const std::string& cmd, CCycCore& core);
void showCycFilterOutput(CCycCore* pCore, CCycFilterBase* pFilter, CycDatablockKey keyOverlayFilter);
void showTextualOutput(CCycFilterBase* pFilter);

void startDatablockPlot(CCycCore* core) { core->startDatablockPlot(); };
void stopDatablockPlot(CCycCore* core) { core->stopDatablockPlot(); };

void showUsage()
{
	printf("\nUsage:\n"
		"App_CycCore [options] config_file\n\n"
		"Options:\n"
        "  --save                       # Save the Datablock to the specified folder\n"
        "  --y                          # Overwrite the save folder if not empty\n"
        "  --core                       # Add publishable filters from another core via conf file\n"
        "  --no-viz                     # Disable visualization of filters output\n"
		"eg: App_CycCore --save c:/data ../etc/cyc_core.conf\n");
	exit(EXIT_SUCCESS);
}

bool can_start_plots(CCycCore& core)
{
#ifdef QTPLOT_HAS_SKELETON
    return true;
#else
    std::vector<ConfigFilterParameters> filters;
    CConfigParameters::instance().getFiltersConfigParameters(filters);

    bool can_start = true;
    for (const auto& params : filters)
    {
        CCycFilterBase* pFilter = nullptr;
        core.readFilter(params.key, pFilter);
        if (pFilter != nullptr)
        {
            if (pFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VIZ_OCTOVIZ_FILTER_TYPE"))
            {
                can_start = can_start && (pFilter->getTimestampStop() > 0);
            }
        }
    }

    return can_start;
#endif // QTPLOT_HAS_SKELETON
}

namespace {
std::atomic<bool> appRunning = true;

void handle_sigint(int sig) {
    appRunning = false;
}

}

CyC_INT main(CyC_INT argc, char** argv)
{
    // Use "." as decimal separator
    std::setlocale(LC_NUMERIC, "C");
    signal(SIGINT, handle_sigint);

#ifdef WIN32
    // Set resolution of 1ms for the interrupt timer
    timeBeginPeriod(1);
#endif // WIN32

    console::cls();
    
    bool                            bVizEnabled = true;
    CCycQTSkeleton*                 qt;
    std::unique_ptr<CCycQTImage>    qt_image;
    bool                            bSaveDatablock(false);
    std::string                     strSaveFolder;
    bool                            bSaveOverwrite(false);
    bool                            bSaveIncremental(false);
    std::vector <std::string>       vectNetworkCoresConfFiles;
    bool                            bStartInitialView(false);

    if (argc < 2)
    {
        showUsage();
        return EXIT_SUCCESS;
    }

    //cv::namedWindow("a");
    //cv::destroyWindow("a");

    for (CyC_INT i = 1; i < argc - 1; i++)
    {
        if (strcmp(argv[i], "--save") == 0)
        {
            bSaveDatablock = true;
            strSaveFolder = argv[i + 1];
            ++i;
            std::cout << "CyberCortex.AI Core: Saving to folder '" << strSaveFolder << "'" << std::endl;
            continue;
        }
        else if (strcmp(argv[i], "--y") == 0)
        {
            bSaveOverwrite = true;
            std::cout << "CyberCortex.AI Core: Overwriting folder '" << strSaveFolder << "'" << std::endl;
            continue;
        }
        else if (strcmp(argv[i], "--i") == 0)
        {
            bSaveIncremental = true;
            std::cout << "CyberCortex.AI Core: Incremental saving in folder '" << strSaveFolder << "'" << std::endl;
            continue;
        }
        else if (strcmp(argv[i], "--core") == 0)
        {
            vectNetworkCoresConfFiles.emplace_back(argv[i + 1]);
            ++i;
            continue;
        }
        else if (strcmp(argv[i], "--no-viz") == 0)
        {
            bVizEnabled = false;
            std::cout << "CyberCortex.AI Core: Disabling QtPlot visualization" << std::endl;
            continue;
        }

        printf("Unrecognized option : %s\n", argv[i]);
        showUsage();
    }

    // Read the config file
    const std::string confFile = argv[argc - 1];

    if (!CFileUtils::FileExist(confFile.c_str()))
    {
        std::cout << "CyberCortex.AI Core: Configuration file not found. Exiting." << std::endl;
        exit(EXIT_FAILURE);
    }

    if (bSaveDatablock)
    {
        // Check if folder exists
        if (!CFileUtils::FolderExist(strSaveFolder.c_str()))
        {
            std::cout << "CyberCortex.AI Core: Storage folder not found. Exiting." << std::endl;
            exit(EXIT_FAILURE);
        }

        // Check if folder is empty
        const fs::path save_dir{ strSaveFolder.c_str() };
        if (!bSaveIncremental && !fs::is_empty(save_dir) && !bSaveOverwrite)
        {
            std::cout << "CyberCortex.AI Core: Storage folder is not empty. Overwrite data? [y/N]." << std::endl;
            char input; std::cin >> input; input = std::tolower(input);
            if (input == 'y')
            {
                bSaveOverwrite = true;
            }
            else
            {
                // Exit
                std::cout << "CyberCortex.AI Core: Exiting." << std::endl;
                exit(EXIT_SUCCESS);
            }
        }

        // Delete folder content
        if (!bSaveIncremental && !fs::is_empty(save_dir) && bSaveOverwrite)
        {
            for (const auto& entry : fs::directory_iterator(save_dir))
                fs::remove_all(entry.path());
            std::cout << "CyberCortex.AI Core: Save folder content deleted." << std::endl;
        }

        // Incremental saving folders
        if (bSaveIncremental)
        {
            const int lastFolderID = CFileUtils::getLatestFolderAsInt(save_dir);
            const fs::path dir_path = fs::path(save_dir) / CStringUtils::padding_int2str(lastFolderID + 1, 6);
            fs::create_directories(dir_path);
            strSaveFolder = dir_path;
        }
    }

    CCycCore CycCore;
    const bool bInitCore = CycCore.init(confFile, vectNetworkCoresConfFiles);

    if (!bInitCore)
    {
        std::cout << std::endl;
        std::cout << "The CyC Core could not be initialized." << std::endl;
        std::cout << "Check the configuration file path." << std::endl;
        std::cout << "Exiting." << std::endl;
        return EXIT_FAILURE;
    }

    // Enable all filters
    CycCore.startAllFilters();

    // Initialize storage
    CHddStorage storage(&CycCore, CycCore.getReplayDBPath(), strSaveFolder);

    // Initialize default filter view
    if (CycCore.getStartupFiltersView().size() > 0)
        bStartInitialView = true;

    // Read Datablock from database
    std::cout << "***** Local Replay Database *****" << std::endl;
    if (!storage.readDatablockSynced())
    {
        std::ostringstream ss;
        ss << "Datablock '" << CycCore.getReplayDBPath() << "' cannot be read. Check log for errors.";
        std::cout << ss.str() << std::endl;
    }
    else
    {
        std::ostringstream ss;
        ss << "Replaying datablock filters from '" << CycCore.getReplayDBPath() << "'.";
        std::cout << ss.str() << std::endl;
    }

    // Save Datablock code
    if (bSaveDatablock)
    {
        std::cout << "Saving Datablock ..." << std::endl;

        if (storage.saveDatablockSynced())
            std::cout << "Datablock saved" << std::endl;
        else
            std::cout << "Datablock cannot be saved. Check log for errors." << std::endl;
    }

    console::setup();
    console::enable_raw_mode();

    char ch = 0;
    CCycFilterBase* pTextualFilter = nullptr;
    std::stringstream cmd_stream;

    // Enable visualization
    if (bVizEnabled)
    {
        qt = CycCore.getSingletonRegistry()->get<CCycQTSkeleton>().get();
        qt_image = std::make_unique<CCycQTImage>(qt);
    }

    while (((cmd_stream.tellg() > 0) || (ch != 27)) && appRunning) // 27 = ESC
    {
        console::cursor_guard guard;

        CycCore.printDatablock();

        if (pTextualFilter != nullptr)
        {
            showTextualOutput(pTextualFilter);
        }

        std::cout << "Enter command: " << std::flush;
        std::cout << cmd_stream.str() << std::flush;

        int filterID = std::numeric_limits<int>::min();

        g_iq_mutex.lock();
        if (!g_image_queue.empty())
        {
            auto val = g_image_queue.back();
            // TODO
            //g_image_queue.pop();
            while (!g_image_queue.empty())
                g_image_queue.pop();
            if (window_names_map.find(val.filterID) != window_names_map.end())
            {
                cv::Mat cvt;
                cv::cvtColor(val.img, cvt, cv::COLOR_BGR2RGBA);
                if (qt_image->display_rgba(cvt.data, cvt.cols, cvt.rows, window_names_map[val.filterID]) == -2)
                    window_names_map.erase(val.filterID);

                filterID = val.filterID;
            }
        }
        g_iq_mutex.unlock();

        if (filterID != std::numeric_limits<int>::min())
        {
            //const ROVIS_INT delay = (ROVIS_INT)(33 / window_names_map.size());
            const CyC_INT delay = 5; // prevent slow memory leak caused by the queue being emptied slower than being filled

            if (cv::waitKey(delay) > 0)
            {
                visualization_running_map[filterID] = false;
                if (visualization_thread_map[filterID].joinable())
                {
                    visualization_thread_map[filterID].join();
                }

                cv::destroyWindow(window_names_map[filterID]);
                window_names_map.erase(filterID);
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }

        // Initial view
        if (bVizEnabled && bStartInitialView && can_start_plots(CycCore))
        {
            for (const CyC_INT& id : CycCore.getStartupFiltersView())
            {
                bStartInitialView = false;
                pTextualFilter = processCommand(std::to_string(id), CycCore);
            }
        }

        if (console::kbhit())
        {
            ch = console::gc();

            switch (ch)
            {
            case 10: // enter
                if (bVizEnabled)
                    pTextualFilter = processCommand(cmd_stream.str(), CycCore);

                cmd_stream.str("");
                ch = 0;
                break;

            case 27: // esc
                if (!cmd_stream.str().empty())
                {
                    cmd_stream.str("");
                    ch = 0;
                }
                break;

            default:
                //if (isalnum(ch))
            {
                cmd_stream << ch;
            }
            break;
            }
        }
    }


    console::disable_raw_mode();
    console::restore();

    for (auto& viz_pair : visualization_running_map)
    {
        viz_pair.second = false;
    }

    for (auto& viz_pair : visualization_thread_map)
    {
        if (viz_pair.second.joinable())
        {
            viz_pair.second.join();
        }
    }

    return EXIT_SUCCESS;
}

std::vector<CyC_UINT> parseCommand(const std::string& cmd)
{
    csv::reader::row r;
    r.parse_line(cmd, ' ');

    std::vector<CyC_UINT> out;
    for (size_t i = 0; i < r.size(); ++i)
    {
        out.push_back(r.get<CyC_UINT>(i));
    }

    return out;
}

CCycFilterBase* processCommand(const std::string& cmd, CCycCore& core)
{
    if (cmd.empty())
		return nullptr;

    if (isdigit(cmd[0]))
	{
        const auto cmds = parseCommand(cmd);
        CyC_UINT nCoreID = core.getVisionCoreID();
		CyC_UINT nFilterID = cmds[0];

        if (cmds.size() > 1)
        {
            nCoreID = cmds[0];
            nFilterID = cmds[1];
        }
        
		CCycFilterBase* pFilter = nullptr;
		CycDatablockKey key(nCoreID, nFilterID);
		const bool bFilterRead = core.readFilter(key, pFilter);
        
        // TODO: remove this hardcode
        const CyC_UINT nOverlayFilterID = 1; // (cmds.size() > 1) ? cmds[1] : 1;
        CycDatablockKey keyOverlayFilter(core.getVisionCoreID(), nOverlayFilterID);

		if (bFilterRead)
		{
			switch (pFilter->getOutputDataType())
			{
			    case CyC_IMAGE:
                case CyC_VOXELS:
			    case CyC_POSES_6D:
			    case CyC_3D_BBOXES:
                case CyC_2D_ROIS:
                case CyC_POINTS:
			    case CyC_LANES_MODEL:
                case CyC_STATE:
			    case CyC_CONTROL_INPUT:
                case CyC_IMU:
                case CyC_GPS:
                case CyC_VECTOR_INT:
                case CyC_VECTOR_FLOAT:
                case CyC_ULTRASONICS:
				    break; // not a textual output
			    default:
				    return pFilter; // textual output
			}

			auto& visualization_running = visualization_running_map[nFilterID];
			auto& visualization_thread = visualization_thread_map[nFilterID];
			if (visualization_running)
			{
				visualization_running = false;
				visualization_thread.join();
			}

			visualization_running = true;
			new (&visualization_thread) std::thread{ showCycFilterOutput, &core, pFilter, keyOverlayFilter };
		}
	}
	else
	{
		switch (cmd[0])
		{
            case 't':
            {
                const CyC_INT nFilterID = -2; // fake id, unique for signals timing plotting!!
                
                auto& visualization_running = visualization_running_map[nFilterID];
                auto& visualization_thread = visualization_thread_map[nFilterID];
                window_names_map[nFilterID] = "Datablock signals";

                if (visualization_running)
                {
                    visualization_running = false;
                    visualization_thread.join();
                }

                visualization_running = true;
                new (&visualization_thread) std::thread{ startDatablockPlot, &core };
            }
            break;
            default:
            break;
		}
	}

	return nullptr;

	// TODO:
	//  - handle more commands?
	//  - implement the overlayfilter thing
}

void displayImage(CyC_UINT filterID, const cv::Mat& img)
{
    g_iq_mutex.lock();
    g_image_queue.push(image_queue_vtype{filterID, img});
    g_iq_mutex.unlock();
}

void displayImage(CCycFilterBase* pFilter, const cv::Mat& img)
{
    displayImage(pFilter->getFilterKey().nFilterID, img);
}

// Plots a filter output that is a plain vector of numeric values (CyC_VECTOR_INT / CyC_VECTOR_FLOAT).
// One plot segment is created per vector element, using the vector size from the first sample read.
template <typename T>
void plotCycVectorOutput(CCycCore* pCore, CCycFilterBase* pFilter, bool& visualization_running, const std::string& sIdentifier)
{
    CyC_TIME_UNIT lastReadTime = 0;
    bool bInitialized(false);
    CCcrQTPlot* qt_plot = nullptr;

    while (visualization_running)
    {
        auto readTime = pFilter->getTimestampStop();

        if (readTime > lastReadTime)
        {
            std::vector<T> data;
            bool bDataRead = pFilter->getData(data);

            if (!bInitialized && bDataRead)
            {
                qt_plot = new CCcrQTPlot(sIdentifier + ": " + pFilter->getFilterName(), pCore->getSingletonRegistry()->get<CCycQTSkeleton>().get());
                for (size_t i = 0; i < data.size(); ++i)
                    qt_plot->add_segment("Data " + std::to_string(i), {});

                qt_plot->run();
                bInitialized = true;
            }

            if (bInitialized)
            {
                std::vector<float> signals_values;
                for (size_t i = 0; i < data.size(); ++i)
                    signals_values.push_back((float)data[i]);

                qt_plot->plot_signals(signals_values);
            }

            lastReadTime = readTime;
        }
    }

    // Free memory
    if (qt_plot != nullptr)
    {
        qt_plot->stop();
        delete qt_plot;
    }
}

void showCycFilterOutput(CCycCore* pCore, CCycFilterBase* pFilter, CycDatablockKey keyOverlayFilter)
{
    CyC_INT nr_of_fps = 30;
    float time_scale = 1.3f;
    CPlotFilterOutput plotFilterOutput(pCore);
    
    auto& visualization_running = visualization_running_map[pFilter->getFilterKey().nFilterID];

	if (pFilter->isRunning())
	{
        const std::string sIdentifier = "CoreID: " + std::to_string(pCore->getVisionCoreID()) + " FilterID: " + std::to_string(pFilter->getFilterKey().nFilterID);

		const std::string sWindowName = " output image; " + sIdentifier;
		window_names_map[pFilter->getFilterKey().nFilterID] = sWindowName;

		switch (pFilter->getOutputDataType())
		{
		    case CyC_IMAGE:
		    {
		        CyC_TIME_UNIT lastReadTime = 0;
            
			    while (visualization_running)
			    {
				    cv::Mat cvDispImg;
				    auto readTime = pFilter->getTimestampStop();

				    if (readTime > lastReadTime)
                    {
                        lastReadTime = readTime;
                        if (pFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_SEMANTIC_SEGMENTATION_FILTER_TYPE"))
                        {
                            if (plotFilterOutput.plotCcrSemanticSegmentationImage(cvDispImg, pFilter))
                                displayImage(pFilter, cvDispImg);
                        }
                        else
                        {
                            if (plotFilterOutput.plotCycImage(cvDispImg, pFilter))
                                displayImage(pFilter, cvDispImg);
                        }
				    }

				    std::this_thread::sleep_for(std::chrono::milliseconds(1));
			    }
		    }
		    break;

            case CyC_ULTRASONICS:
            {
                CyC_TIME_UNIT lastReadTime = 0;
                while (visualization_running)
                {
                    cv::Mat cvDispImg;
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        lastReadTime = readTime;
                        if (plotFilterOutput.plotUltrasonics(cvDispImg, pFilter))
                            displayImage(pFilter, cvDispImg);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            break;

            case CyC_POINTS:
            {
                CyC_TIME_UNIT lastReadTime = 0;
                while (visualization_running)
                {
                    cv::Mat cvDispImg;
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        lastReadTime = readTime;

                        if (plotFilterOutput.plotCycPoints(cvDispImg, pFilter))
                            displayImage(pFilter, cvDispImg);
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            break;

            case CyC_2D_ROIS:
            {
                std::vector<std::string> strLabels;

                CyC_TIME_UNIT lastReadTime = 0;
                while (visualization_running)
                {
                    cv::Mat cvDispImg;
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        lastReadTime = readTime;

                        if (plotFilterOutput.plotCycRoi2D(cvDispImg, pFilter))
                            displayImage(pFilter, cvDispImg);
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            break;

            case CyC_LANES_MODEL:
            {
                CyC_TIME_UNIT lastReadTime = 0;
                while (visualization_running)
                {
                    cv::Mat cvDispImg;
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        lastReadTime = readTime;

                        if (plotFilterOutput.plotCycLanes(cvDispImg, pFilter))
                            displayImage(pFilter, cvDispImg);
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            break;

            case CyC_VOXELS:
            {
                std::vector<std::string> strLabels;
            
                CyC_TIME_UNIT lastReadTime = 0;
                while (visualization_running)
                {
                    cv::Mat cvDispImg;
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        lastReadTime = readTime;

                        if (plotFilterOutput.plotCycVoxels(cvDispImg, pFilter))
                            displayImage(pFilter, cvDispImg);
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            break;

		    case CyC_POSES_6D:
		    {
			    CyC_TIME_UNIT lastReadTime = 0;
			    while (visualization_running)
			    {
			        auto readTime = pFilter->getTimestampStop();

			        if (readTime > lastReadTime)
                    {
                        cv::Mat cvDispImg;
                        auto readTime = pFilter->getTimestampStop();

                        if (readTime > lastReadTime)
                        {
                            lastReadTime = readTime;

                            if (plotFilterOutput.plotCycPoses(cvDispImg, pFilter))
                                displayImage(pFilter, cvDispImg);
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                }
		    }
		    break;

            case CyC_3D_BBOXES: // 3d bboxes projected in 2d
            {
                CyC_TIME_UNIT lastReadTime = 0;
                while (visualization_running)
                {
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        cv::Mat cvDispImg;
                        auto readTime = pFilter->getTimestampStop();

                        if (readTime > lastReadTime)
                        {
                            lastReadTime = readTime;

                            if (plotFilterOutput.plotCycBBox3D(cvDispImg, pFilter, keyOverlayFilter))
                                displayImage(pFilter, cvDispImg);
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                }
            }
            break;

            case CyC_STATE:
            {
                CyC_TIME_UNIT lastReadTime = 0;
                std::vector<std::string> signals_names;

                // Wait until data is present
                bool bDataRead = false;
                while (!bDataRead)
                {
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        CycState state;
                        if (pFilter->getData(state))
                        {
                            // visualize drone state [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel] ^ T
                            for (CyC_INT i = 0; i < state.x_hat.size(); ++i)
                                signals_names.push_back("State " + std::to_string(i));

                            bDataRead = true;
                        }
                    }
                }

                CCcrQTPlot plot(sIdentifier + ": " + pFilter->getFilterName(), pCore->getSingletonRegistry()->get<CCycQTSkeleton>().get());
                for (const std::string& name : signals_names)
                    plot.add_segment(name, {});
                plot.run();
                while (visualization_running)
                {
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        CycState state;
                        bool bDataRead = pFilter->getData(state);

                        std::vector<float> signals_values;

                        for (CyC_INT i = 0; i < state.x_hat.size(); ++i)
                            signals_values.push_back(state.x_hat(i));

                        plot.plot_signals(signals_values);

                        lastReadTime = readTime;
                    }
                }
                plot.stop();
            }
            break;

		    case CyC_CONTROL_INPUT:
		    {
                std::vector<std::string> signals_names;
                CyC_TIME_UNIT lastReadTime = 0;
                
                // Wait until data is present
                bool bDataRead = false;
                while (!bDataRead)
                {
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        CycControlInput cmd;
                        if (pFilter->getData(cmd))
                        {
                            // visualize drone state [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel] ^ T
                            for (CyC_INT i = 0; i < cmd.u.size(); ++i)
                                signals_names.push_back("Control input " + std::to_string(i));

                            bDataRead = true;
                        }
                    }
                }
            
                CCcrQTPlot plot(sIdentifier + ": " + pFilter->getFilterName(), pCore->getSingletonRegistry()->get<CCycQTSkeleton>().get());
                for (const std::string& name : signals_names)
                    plot.add_segment(name, {});
                plot.run();
                while (visualization_running)
                {
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        CycControlInput cmd;
                        bool bDataRead = pFilter->getData(cmd);

                        std::vector<float> signals_values;
                        for (CyC_INT i = 0; i < cmd.u.size(); ++i)
                            signals_values.push_back(cmd.u(i));

                        plot.plot_signals(signals_values);

                        lastReadTime = readTime;
                    }
                }
                plot.stop();
		    }
		    break;

            case CyC_IMU:
            {
                std::vector<std::string> signals_names;
                signals_names.push_back("X acc. [m/s^2]");
                signals_names.push_back("Y acc. [m/s^2]");
                signals_names.push_back("Z acc. [m/s^2]");
                signals_names.push_back("X ang. vel. [deg/s]");
                signals_names.push_back("Y ang. vel. [deg/s]");
                signals_names.push_back("Z ang. vel. [deg/s]");
                signals_names.push_back("X magnet [mT]");
                signals_names.push_back("Y magnet [mT]");
                signals_names.push_back("Z magnet [mT]");

                CCcrQTPlot plot(sIdentifier + ": " + pFilter->getFilterName(), pCore->getSingletonRegistry()->get<CCycQTSkeleton>().get());
                for (const std::string& name : signals_names)
                    plot.add_segment(name, {});
                plot.run();
                CyC_TIME_UNIT lastReadTime = 0;

                while (visualization_running)
                {
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        CycImus imu_data;
                        bool bDataRead = pFilter->getData(imu_data);

                        for (const auto& imu : imu_data)
                        {
                            std::vector<float> signals_values;
                            signals_values.push_back(imu.acc.x());
                            signals_values.push_back(imu.acc.y());
                            signals_values.push_back(imu.acc.z());
                            signals_values.push_back(imu.gyro.x());
                            signals_values.push_back(imu.gyro.y());
                            signals_values.push_back(imu.gyro.z());
                            signals_values.push_back(imu.magnet.x());
                            signals_values.push_back(imu.magnet.y());
                            signals_values.push_back(imu.magnet.z());
                            plot.plot_signals(signals_values);
                        }
                        
                        lastReadTime = readTime;
                    }
                }
                plot.stop();
            }
            break;

            case CyC_GPS:
            {
                std::vector<std::string> signals_names;
                signals_names.push_back("Latitude");
                signals_names.push_back("Longitude");
                signals_names.push_back("Altitude");
            
                CCcrQTPlot plot(sIdentifier + ": " + pFilter->getFilterName(), pCore->getSingletonRegistry()->get<CCycQTSkeleton>().get());
                for (const std::string& name : signals_names)
                    plot.add_segment(name, {});
                plot.run();

                CyC_TIME_UNIT lastReadTime = 0;

                while (visualization_running)
                {
                    auto readTime = pFilter->getTimestampStop();

                    if (readTime > lastReadTime)
                    {
                        CycGps gps;
                        bool bDataRead = pFilter->getData(gps);

                        std::vector<float> signals_values;
                        signals_values.push_back(gps.latitude);
                        signals_values.push_back(gps.longitude);
                        signals_values.push_back(gps.altitude);

                        plot.plot_signals(signals_values);

                        lastReadTime = readTime;
                    }
                }
                plot.stop();
            }
            break;

            case CyC_VECTOR_INT:
                plotCycVectorOutput<CyC_INT>(pCore, pFilter, visualization_running, sIdentifier);
                break;

            case CyC_VECTOR_FLOAT:
                plotCycVectorOutput<float>(pCore, pFilter, visualization_running, sIdentifier);
                break;

		    default:
			break;
		}
	}
}

void showTextualOutput(CCycFilterBase* pFilter)
{
    if (pFilter->isRunning())
	{
		std::cout << "Output of filter #" << pFilter->getFilterKey().nCoreID << "-" << pFilter->getFilterKey().nFilterID << ": " << std::endl;

		switch (pFilter->getOutputDataType())
		{
		case CyC_VOXELS:
		{
			CycVoxels voxels;
			bool bDataRead = pFilter->getData(voxels);
			if (bDataRead)
			{
                for (auto voxel : voxels)
				    std::cout << "voxel: " << voxel.pt3d.x() << " " << voxel.pt3d.y() << " " << voxel.pt3d.z() << " " << voxel.pt3d.w() << std::endl;
			}
		}
		break;

		case CyC_GPS:
		{
            CycGps gps;
			bool bDataRead = pFilter->getData(gps);

			if (bDataRead)
			{
				std::cout << "gps: " << gps.latitude << " " << gps.longitude << " " << gps.altitude << std::endl;
			}
		}
		break;

		case CyC_IMU:
		{
			CycImu imu;
			bool bDataRead = pFilter->getData(imu);

			if (bDataRead)
			{
				std::cout << "imu: " << imu << std::endl;
			}
		}
		break;

		case CyC_2D_ROIS:
		{
			CycRois2D bboxes;
			bool bDataRead = pFilter->getData(bboxes);

			if (bDataRead)
			{
				for (size_t i = 0; i < bboxes.size(); ++i)
				{
					std::cout << "tracked bbox: " << bboxes[i].id << " x" << bboxes[i].origin.x() << " y" << bboxes[i].origin.y() << " widht" << bboxes[i].width << " height" << bboxes[i].height << std::endl;
				}
			}
		}
		break;
		default:
			break;
		}
	}
}
