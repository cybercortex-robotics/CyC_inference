// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CHddStorage.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iomanip>
#include "os/CCsvReader.h"
#include "os/CyC_FILESYSTEM.h"
#include <vision/CDepthImageProcessing.h>

CHddStorage::CHddStorage(CCycCore* _core, const std::string& _read_path, const std::string& _save_path) :
    m_pCycCore(_core),
    m_sReadDBStoragePath(_read_path),
    m_sSaveDBStoragePath(_save_path),
    m_bReadDBEnabled(false),
    m_bSaveReadSamePath(false),
    m_save_thread_pool(std::thread::hardware_concurrency()),
    m_save_running(false),
    m_read_thread_pool(std::thread::hardware_concurrency()),
    m_read_running(false)
{
    if (_core == nullptr)
    {
        spdlog::error("CHddStorage: CycCore undefined. Exiting.");
        exit(EXIT_FAILURE);
    }

    // Check if the read database exists
    if (!fs::exists(_read_path))
    {
        spdlog::error("CHddStorage: CCR database \'{}\' does not exist. Disabling data reading.", _read_path);
    }
    else
    {
        // Check if the read and save databases paths are the same
        if (fs::exists(_save_path) && fs::equivalent(_read_path, _save_path))
        {
            spdlog::error("CHddStorage: reading and saving paths cannot be the same. Disabling data saving.");
            m_bSaveReadSamePath = true;
        }
        else
        {
            m_bSaveReadSamePath = false;
        }
    }
}

CHddStorage::~CHddStorage()
{
    for (std::thread &store_thread : m_vSavingThreads)
        if (store_thread.joinable())
            store_thread.join();
}

void setup_directories(CCycFilterBase* pFilter, const std::string& base_dir)
{
    // Write the objects classes map
    if (!pFilter->m_CustomParameters["object_classes"].empty())
    {
        if (CFileUtils::FileExist(pFilter->m_CustomParameters["object_classes"].c_str()))
        {
            fs::path src = pFilter->m_CustomParameters["object_classes"];
            fs::path dst = base_dir + "/object_classes.conf";
            fs::copy_file(src, dst, fs::copy_options::overwrite_existing);
        }
    }

    switch (pFilter->getOutputDataType())
    {
        case CyC_IMAGE:
        {
            CycImages cycImages;
            while (!pFilter->getData(cycImages)) // make sure data is read in order to write the header
                std::this_thread::sleep_for(std::chrono::microseconds(1));

            for (size_t i = 0; i < cycImages.size(); ++i)
            {
                fs::create_directories(base_dir + "/samples/" + std::to_string(i) + "/left");
                fs::create_directories(base_dir + "/samples/" + std::to_string(i) + "/right");
            }
        }
        break;
        case CyC_IMU:
        {
            fs::create_directories(base_dir + "/samples/");
        }
        break;
        case CyC_OCTREE:
        {
            fs::create_directories(base_dir + "/samples/");
        }
        break;
        case CyC_VOXELS:
        {
            fs::create_directories(base_dir + "/samples/");
        }
        break;
        default:
        {
            spdlog::info("No extra directories needed for filter {}.", pFilter->getFilterName());
        }
        break;
    }
}

void write_header(CCycFilterBase* pFilter, std::ofstream& writer)
{
    switch (pFilter->getOutputDataType())
    {
    case CyC_REFERENCE_SETPOINTS:
    {
        //CycReferenceSetPoints refSetpoints;
        //while (!pFilter->getData(refSetpoints)) // make sure data is read
        //    std::this_thread::sleep_for(std::chrono::microseconds(1));
        writer << "timestamp_stop,sampling_time,frame_id";
        writer << std::endl;
    }
    break;
    case CyC_CONTROL_INPUT:
    {
        CycControlInput controlInput;
        while (!pFilter->getData(controlInput)) // make sure data is read
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        writer << "timestamp_stop,sampling_time";
        for (CyC_UINT idx = 0; idx < controlInput.u.size(); ++idx)
        {
            writer << "," << "cmd_" << idx;
        }
        writer << std::endl;
    }
    break;
    case CyC_IMAGE:
    {
        CycImages cycImages;
        while (!pFilter->getData(cycImages)) // make sure data is read in order to write the header
            std::this_thread::sleep_for(std::chrono::microseconds(1));

        writer << "timestamp_stop,sampling_time";

        for (size_t i = 0; i < cycImages.size(); ++i)
        {
            writer << ",timestamp_image_" << i << ",left_file_path_" << i << ",right_file_path_" << i;
        }

        writer << std::endl;
    }
    break;
    case CyC_IMU:
    {
        writer << "timestamp_stop,sampling_time,imu_file_path" << std::endl;
    }
    break;
    case CyC_GPS:
    {
        writer << "timestamp_stop,sampling_time,lat,lng,alt" << std::endl;
    }
    break;
    case CyC_STATE:
    {
        CycState state;
        while (!pFilter->getData(state)) // make sure data is read in order to write the header
            std::this_thread::sleep_for(std::chrono::microseconds(1));

        writer << "timestamp_stop,sampling_time";

        for (CyC_UINT i = 0; i < state.x_hat.size(); ++i)
            writer << ",state_variable_" << i;

        writer << std::endl;
    }
    break;
    case CyC_2D_ROIS:
    {
        writer << "timestamp_stop,sampling_time,frame_id" << std::endl;
    }
    break;
    case CyC_POINTS:
    {
        writer << "timestamp_stop,sampling_time,frame_id" << std::endl;
    }
    break;
    case CyC_VECTOR_INT:
    {
        std::vector<CyC_INT> data;
        while (!pFilter->getData(data)) // make sure data is read in order to write the header
            std::this_thread::sleep_for(std::chrono::microseconds(1));

        writer << "timestamp_stop,sampling_time";

        for (CyC_UINT i = 0; i < data.size(); ++i)
            writer << ",data_" << i;

        writer << std::endl;
    }
    break;
    case CyC_VECTOR_FLOAT:
    {
        std::vector<float> data;
        while (!pFilter->getData(data)) // make sure data is read in order to write the header
            std::this_thread::sleep_for(std::chrono::microseconds(1));

        writer << "timestamp_stop,sampling_time";

        for (CyC_UINT i = 0; i < data.size(); ++i)
            writer << ",data_" << i;

        writer << std::endl;
    }
    break;
    case CyC_OCTREE:
    {
        //CycEnvironment data(1.f);
        //while (!pFilter->getData(data)) // make sure data is read in order to write the header
        //    std::this_thread::sleep_for(std::chrono::microseconds(1));

        writer << "timestamp_stop,sampling_time,octree_file_path_" << std::endl;
    }
    break;
    case CyC_VOXELS:
    {
        //CycVoxels data;
        //while (!pFilter->getData(data)) // make sure data is read in order to write the header
        //    std::this_thread::sleep_for(std::chrono::microseconds(1));

        writer << "timestamp_stop,sampling_time,voxels_file_path" << std::endl;
    }
    break;
    case CyC_POSES_6D:
    {
        writer << "timestamp_stop,sampling_time,frame_id" << std::endl;
    }
    break;
    case CyC_VECTOR_STRING:
    {
        writer << "timestamp_stop,sampling_time,frame_id" << std::endl;
    }
    break;
    case CyC_TERMINAL_DATA:
    {
        writer << "timestamp_stop,sampling_time,cmd,mode" << std::endl;
    }
    break;
    default:
    {
        spdlog::error("Header not set for output data type {}.", static_cast<int>(pFilter->getOutputDataType()));
    }
    break;
    }
}

bool CHddStorage::saveDatablockAsync()
{
    if (m_bSaveReadSamePath)
    {
        spdlog::error("CHddStorage::saveDatablockAsync(): reading and saving paths cannot be the same. Data saving is disabled.");
        return false;
    }

    // Free memory
    for (auto& mutex : m_mapSavingMutexes)
        delete mutex.second;
    m_mapSavingMutexes.clear();
    m_mapSyncSavingTimestamps.clear();
    m_mapIsSaving.clear();

    // Create the storage folder
    if (fs::exists(m_sSaveDBStoragePath))
        fs::remove_all(m_sSaveDBStoragePath);
    fs::create_directories(m_sSaveDBStoragePath);

    // Create CyC Datablock csv descriptor file
    generateDatablockDescriptor();

    // Access filters
    std::vector<CCycFilterBase*> filters;
    auto Datablockinfo = m_pCycCore->getDatablock();

    for (auto &el : Datablockinfo)
    {
        //const CycDatablockKey filterKey{ el.coreID, el.filterID };
        CCycFilterBase *pFilter = nullptr;

        if (m_pCycCore->readFilter(el.Key, pFilter))
            filters.push_back(pFilter);
    }
    
    // Asynchronous data saving
    for (auto &filter : filters)
    {
        //m_vbSaving.push_back(false);
        std::string sDatastreamPath = m_sSaveDBStoragePath + "/datastream_" + std::to_string(filter->getFilterKey().nFilterID);

        generateFilterOutputStructures(filter, sDatastreamPath);
        startDataSavingThreads(filter, sDatastreamPath);
    }

    return true;
}

bool CHddStorage::readDatablockSynced()
{
    if (m_read_thread.joinable())
    {
        m_read_running = false;
        m_read_thread.join();
    }

    if (!fs::exists(m_sReadDBStoragePath))
    {
        spdlog::error("CHddStorage::readDatablockSynced(): CCR database {} does not exists. Replay from database disabled.", m_sReadDBStoragePath);
        return false;
    }

    spdlog::info("CHddStorage::readDatablockSynced(): Replay database folder: {}", m_sReadDBStoragePath);

    std::vector<CCycFilterBase*> filters;

    for (const auto& binfo : m_pCycCore->getDatablock())
    {
        //const CycDatablockKey filterKey{ binfo.coreID, binfo.filterID };
        CCycFilterBase* pFilter = nullptr;

        if (m_pCycCore->readFilter(binfo.Key, pFilter) && pFilter->isReplayFilter())
            filters.emplace_back(pFilter);
    }

    if (filters.empty())
    {
        spdlog::warn("CHddStorage::readDatablockSynced(): No replay filters found.");
        return false;
    }

    csv::reader csv_Datablock_descriptor;
    if (!csv_Datablock_descriptor.open(m_sReadDBStoragePath + "/datablock_descriptor.csv"))
    {
        spdlog::error("CHDDStorage::readDatablockSynced(): Failed to open csv {}", m_sReadDBStoragePath + "/datablock_descriptor.csv");
        return false;
    }

    csv_Datablock_descriptor.select_cols("core_id", "filter_id");
    std::vector<std::pair<CyC_INT, CyC_INT>> available_filter_datastreams;

    CyC_INT core_id, filter_id;
    while (csv_Datablock_descriptor.read_row(core_id, filter_id))
    {
        available_filter_datastreams.emplace_back(core_id, filter_id);
    }

    csv::reader synced_csv_reader;
    if (!synced_csv_reader.open(m_sReadDBStoragePath + "/sampling_timestamps_sync.csv"))
    {
        spdlog::error("CHddStorage::readDatablockSynced(): Failed to read synced timestamps csv.");
        return false;
    }

    std::unordered_map<CyC_UINT, CCycFilterBase*> available_read_filters;
    std::unordered_map<CyC_UINT, std::ifstream> datastream_readers;
    std::unordered_map<CyC_UINT, std::string> db_root_paths;
    std::unordered_map<size_t, CyC_UINT> column2filter_map;

    available_read_filters.reserve(available_filter_datastreams.size());
    datastream_readers.reserve(available_filter_datastreams.size());

    for (CCycFilterBase* pFilter : filters)
    {
        const auto datastream_exists = std::any_of(
            available_filter_datastreams.begin(),
            available_filter_datastreams.end(),
            [&](const std::pair<CyC_INT, CyC_INT>& datastream_pair) {
            return (pFilter->getReplayFilter() == datastream_pair.second);
        });

        if (datastream_exists)
        {
            const std::string datastream_name = fmt::format("datastream_{}", pFilter->getReplayFilter());
            const std::string descriptor_path = m_sReadDBStoragePath + "/" + datastream_name + "/data_descriptor.csv";

            std::string datastream_path = m_sReadDBStoragePath + "/" + datastream_name;

            const auto col_iterator = std::find(
                synced_csv_reader.get_column_names().begin(),
                synced_csv_reader.get_column_names().end(),
                datastream_name);

            if (col_iterator == synced_csv_reader.get_column_names().end())
            {
                spdlog::error("CHddStorage::readDatablockSynced(): Datastream for filter {}:{} is not synced.", pFilter->getFilterKey().nCoreID, pFilter->getFilterKey().nFilterID);
                continue;
            }

            if (fs::exists(datastream_path) && fs::exists(descriptor_path))
            {
                std::ifstream datastream_reader;
                datastream_reader.open(descriptor_path);
                std::string line;

                if (datastream_reader.is_open() && std::getline(datastream_reader, line))
                {
                    available_read_filters.emplace(pFilter->getFilterKey().nFilterID, pFilter);
                    datastream_readers.emplace(pFilter->getFilterKey().nFilterID, std::move(datastream_reader));
                    column2filter_map.emplace(std::distance(synced_csv_reader.get_column_names().begin(), col_iterator), pFilter->getFilterKey().nFilterID);
                    db_root_paths.emplace(pFilter->getFilterKey().nFilterID, datastream_path + "/");
                }
                else
                {
                    spdlog::error("CHddStorage::readDatablockSynced(): Failed to read {}.", descriptor_path);
                }
            }
            else
            {
                spdlog::error("CHddStorage::readDatablockSynced(): Datastream does not exists for filter {}:{}", pFilter->getFilterKey().nCoreID, pFilter->getFilterKey().nFilterID);
            }
        }
    }

    if (available_read_filters.empty())
    {
        spdlog::warn("CHddStorage::readDatablockSynced(): No available filters to read.");
        return false;
    }

    if (!synced_csv_reader.next_row())
    {
        spdlog::error("CHddStorage::readDatablockSynced(): Unable to read first row from csv.");
        return false;
    }

    m_read_running = true;
    m_read_thread = std::thread([this](
        csv::reader&& synced_csv_reader,
        std::unordered_map<CyC_UINT, CCycFilterBase*>&& available_read_filters,
        std::unordered_map<CyC_UINT, std::ifstream>&& datastream_readers,
        std::unordered_map<CyC_UINT, std::string>&& db_root_paths,
        std::unordered_map<size_t, CyC_UINT>&& column2filter_map)
        {
            auto last_synced_timestamp = synced_csv_reader.get_row().get<CyC_TIME_UNIT>(0);
            while (m_read_running && synced_csv_reader.next_row())
            {
                const csv::reader::row& row = synced_csv_reader.get_row();
                for (size_t i = 1; i < row.size(); ++i)
                {
                    if (column2filter_map.find(i) == column2filter_map.end())
                    {
                        continue;
                    }

                    const auto filter_timestamp = row.get<CyC_TIME_UNIT>(i);
                    std::string line;
                    if (filter_timestamp != CyC_TIME_UNIT(-1) && std::getline(datastream_readers[column2filter_map[i]], line))
                    {
                        CCycFilterBase* pFilter = available_read_filters[column2filter_map[i]];
                        const std::string& db_root_path = db_root_paths[column2filter_map[i]];
                        m_read_thread_pool.enqueue([i, pFilter, db_root_path](const std::string& line) {
                            pFilter->loadFromDatastream(line, db_root_path);
                        }, std::move(line));
                    }
                }

                const auto current_synced_timestamp = row.get<CyC_TIME_UNIT>(0);
                std::this_thread::sleep_for(std::chrono::milliseconds(current_synced_timestamp - last_synced_timestamp));
                //std::this_thread::sleep_for(std::chrono::microseconds(current_synced_timestamp - last_synced_timestamp));
                last_synced_timestamp = current_synced_timestamp;
            }
        },

        std::move(synced_csv_reader),
        std::move(available_read_filters),
        std::move(datastream_readers),
        std::move(db_root_paths),
        std::move(column2filter_map));

    return true;
}

void CHddStorage::generateDatablockDescriptor()
{
    CycDatablockEntriesInfo Datablockinfo = m_pCycCore->getDatablock();

    // Check if any filter is marked for saving
    bool bSave = false;
    for (auto& el : Datablockinfo)
    {
        CCycFilterBase* pFilter = nullptr;
        if (m_pCycCore->readFilter(el.Key, pFilter) && pFilter->isSave())
            bSave = true;
    }
    if (!bSave)
        return;

    std::ofstream CsvWritter(m_sSaveDBStoragePath + "/datablock_descriptor.csv");
    CsvWritter << "core_id,filter_id,name,type,output_data_type,input_sources" << std::endl;
    CsvWritter.flush();

    std::vector<CCycFilterBase*> filters;
    for (auto &el : Datablockinfo)
    {
        //const CycDatablockKey filterKey{ el.coreID, el.filterID };
        CCycFilterBase* pFilter = nullptr;

        if (m_pCycCore->readFilter(el.Key, pFilter) && pFilter->isSave())
        {
            filters.push_back(pFilter);

            // Save filter configuration in the CCR Datablock csv descriptor file
            CsvWritter << el.Key.nCoreID << "," << el.Key.nFilterID << "," <<
                pFilter->getFilterName() << "," << pFilter->getFilterType() << "," << pFilter->getOutputDataType();

            CsvWritter << ",";

            // Write the input sources for each filter
            if (pFilter->getInputSources().size() > 0)
            {
                CsvWritter << "{";

                for (CyC_UINT i = 0; i < pFilter->getInputSources().size(); ++i)
                {
                    CycDatablockKey key = pFilter->getInputSources()[i].SourceKey;
                    CsvWritter << key.nCoreID << "-" << key.nFilterID;

                    if (i < pFilter->getInputSources().size() - 1)
                        CsvWritter << ";";
                }

                CsvWritter << "}";
            }

            CsvWritter << std::endl;
        }
    }

    // Create the sampling timestamps synchronization file header
    std::ofstream CsvSyncWritter(m_sSaveDBStoragePath + "/sampling_timestamps_sync.csv");
    CsvSyncWritter << "timestamp_stop";
    for (auto &filter : filters)
        CsvSyncWritter << ",datastream_" + std::to_string(filter->getFilterKey().nFilterID);
    CsvSyncWritter << std::endl;
    CsvSyncWritter.flush();
}

void CHddStorage::generateFilterOutputStructures(CCycFilterBase* _pFilter, std::string _datastream_storage_folder)
{
    if (!_pFilter->isSave())
        return;

    fs::create_directories(_datastream_storage_folder);
    std::ofstream CsvWritter(_datastream_storage_folder + "/data_descriptor.csv");
    
    // Create storage cvs headers
    switch (_pFilter->getOutputDataType())
    {
    case CyC_IMAGE:
    {
        CycImages cycImages;
        while (!_pFilter->getData(cycImages)) // make sure data is read in order to write the header
            std::this_thread::sleep_for(std::chrono::microseconds(1));

        CsvWritter << "timestamp_stop,sampling_time";

        for (int i = 0; i < cycImages.size(); ++i) {
            fs::create_directories(_datastream_storage_folder + "/samples/" + std::to_string(i) + "/left");
            fs::create_directories(_datastream_storage_folder + "/samples/" + std::to_string(i) + "/right");
            CsvWritter << ",timestamp_image_" + std::to_string(i) + ",left_file_path_" + std::to_string(i) + ",right_file_path_" + std::to_string(i);
        }
        CsvWritter << std::endl;
    }
    break;

    case CyC_IMU:
    {
        fs::create_directories(_datastream_storage_folder + "/samples");
        CsvWritter << "timestamp_stop,sampling_time,imu_file_path" << std::endl;
    }
    break;

    case CyC_GPS:
    {
        CsvWritter << "timestamp_stop,sampling_time,lat,lng,alt" << std::endl;
    }
    break;

    case CyC_STATE:
    {
        CycState state;
        bool bDataRead = _pFilter->getData(state);

        if (bDataRead)
        {
            CsvWritter << "timestamp_stop,sampling_time";

            for (CyC_UINT i = 0; i < state.x_hat.size(); ++i)
                CsvWritter << ",state_variable_" << std::to_string(i);

            CsvWritter << std::endl;
        }
    }
    break;
    }

    CsvWritter.flush();
}

bool CHddStorage::saveDatablockSynced()
{
    if (m_save_thread.joinable())
    {
        m_save_running = false;
        m_save_thread.join();
    }

    if (m_pCycCore == nullptr)
    {
        return false;
    }

    if (!fs::exists(m_sSaveDBStoragePath))
    {
        fs::create_directories(m_sSaveDBStoragePath);
    }

    generateDatablockDescriptor();

    std::vector<CCycFilterBase*> filters_vec;
    filters_vec.reserve(m_pCycCore->getDatablock().size());
    for (const auto& binfo : m_pCycCore->getDatablock())
    {
        //const CycDatablockKey filterKey{ binfo.coreID, binfo.filterID };
        CCycFilterBase* pFilter = nullptr;
        if (m_pCycCore->readFilter(binfo.Key, pFilter) && pFilter->isSave())
            filters_vec.push_back(pFilter);
    }

    if (filters_vec.empty())
    {
        spdlog::error("No filters available. Save cancelled.");
        return false;
    }

    std::vector<std::ofstream> descriptor_writers(filters_vec.size());
    std::vector<std::string> storage_directories(filters_vec.size());
    for (size_t i = 0; i < descriptor_writers.size(); ++i)
    {
        CCycFilterBase* pFilter = filters_vec[i];
        std::ofstream& writer = descriptor_writers[i];
        std::string& storage_dir = storage_directories[i];

        storage_dir =
            fmt::format("{}/datastream_{}",
                m_sSaveDBStoragePath,
                pFilter->getFilterKey().nFilterID);

        std::error_code ec;
        fs::create_directories(storage_dir, ec);

        const auto descriptor_filepath = storage_dir + "/data_descriptor.csv";

        writer.open(descriptor_filepath);
        if (!writer.is_open())
        {
            spdlog::error("Failed to open file for writing: {}", descriptor_filepath);
        }
        else
        {
            m_setup_done[pFilter->getFilterKey()] = false;
            //setup_directories(pFilter, storage_dir);
            //write_header(pFilter, writer);
        }

        // Write calibration file
        if (pFilter->getSensorModel() != nullptr)
        {
            fs::path src = pFilter->getSensorModel()->getCalibrationFile();
            fs::path dst = storage_dir + "/calibration.cal";
            fs::copy_file(src, dst, fs::copy_options::overwrite_existing);
        }
    }

    const bool any_writer_opened = std::any_of(
        descriptor_writers.begin(),
        descriptor_writers.end(),
        [](auto& stream) { return stream.is_open(); });

    if (!any_writer_opened)
    {
        spdlog::error("No data descriptor will be written. Save cancelled.");
        return false;
    }

    std::ofstream writer_timestamps_sync(m_sSaveDBStoragePath + "/sampling_timestamps_sync.csv");
    if (!writer_timestamps_sync.is_open())
    {
        spdlog::error("Failed to open synced sampling file for writing.");
        return false;
    }

    writer_timestamps_sync << "timestamp_stop";

    for (CCycFilterBase* pFilter : filters_vec)
    {
        writer_timestamps_sync << ",datastream_" << pFilter->getFilterKey().nFilterID;
    }

    writer_timestamps_sync << std::endl;

    m_save_running = true;
    m_save_thread = std::thread([this](std::ofstream &&sync_writer, std::vector<CCycFilterBase*>&& filters_vec,
        std::vector<std::ofstream> &&descriptor_writers, std::vector<std::string> &&storage_directories)
    {
        CTimer timerBlock;
        std::vector<bool> writable(filters_vec.size());
        std::vector<std::mutex> mutex_pool(filters_vec.size());

        // If the clock is the to now instead of 0, the replay filters will not be saved
        const CyC_TIME_UNIT time_now = 0; // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::vector<CyC_TIME_UNIT> last_save_timestamp_vec(filters_vec.size());
        std::fill(last_save_timestamp_vec.begin(), last_save_timestamp_vec.end(), time_now);

        while (m_save_running)
        {
            timerBlock.restart();

            const CyC_TIME_UNIT time_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            sync_writer << time_now;

            std::fill(writable.begin(), writable.end(), false);
            for (size_t i = 0; i < filters_vec.size(); ++i)
            {
                CCycFilterBase* pFilter = filters_vec[i];
                const auto filter_timestamp = pFilter->getTimestampStop();
                const auto last_save_timestamp = last_save_timestamp_vec[i];

                writable[i] = (filter_timestamp > last_save_timestamp);
            }

            for (size_t i = 0; i < filters_vec.size(); ++i)
            {
                CCycFilterBase* pFilter = filters_vec[i];
                std::ofstream& writer = descriptor_writers[i];

                const auto filter_timestamp_stop = pFilter->getTimestampStop();
                auto& last_save_timestamp = last_save_timestamp_vec[i];
                const std::string& storage_dir = storage_directories[i];
                std::mutex& m = mutex_pool[i];

                if (writable[i])
                {
                    last_save_timestamp = filter_timestamp_stop;

                    saveFilter(pFilter, writer, storage_dir, m);

                    sync_writer << ',' << filter_timestamp_stop;
                }
                else
                {
                    sync_writer << ",-1";
                }
            }

            sync_writer << std::endl;

            timerBlock.stop();
            const auto passed_us = timerBlock.elapsedMicroseconds();
            // TODO: Remove hard-coded value for sampling_timestamps_sync interval 
            const auto sleep_us = 10000 - passed_us;
            std::this_thread::sleep_for(std::chrono::microseconds(((sleep_us > 0) ? sleep_us : 1)));
        }
    }, std::move(writer_timestamps_sync),
        std::move(filters_vec),
        std::move(descriptor_writers),
        std::move(storage_directories));

    return true;
}

void CHddStorage::startDataSavingThreads(CCycFilterBase* _pFilter, std::string _storage_folder)
{
    auto storage_func = [this](CCycFilterBase *_pFilter, std::string _storage_folder)
    {
        CyC_TIME_UNIT lastTimestampStop = 0;
        std::ofstream CsvWritter(_storage_folder + "/data_descriptor.csv", std::ofstream::app);

        // Save data
        while (_pFilter->isRunning())
        {
            if (_pFilter->getTimestampStop() > lastTimestampStop)
            {
                {
                    std::lock_guard<std::mutex> lock(*m_mapSavingMutexes[_pFilter->getFilterKey()]);
                    lastTimestampStop = _pFilter->getTimestampStop();
                    m_mapIsSaving[_pFilter->getFilterKey()] = true;
                }

                //saveFilter(_pFilter, CsvWritter, _storage_folder);

                {
                    std::lock_guard<std::mutex> lock(*m_mapSavingMutexes[_pFilter->getFilterKey()]);
                    m_mapIsSaving[_pFilter->getFilterKey()] = false;
                    m_mapSyncSavingTimestamps[_pFilter->getFilterKey()] = CTimer::now();
                }
            }

            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    };

    m_vSavingThreads.push_back(std::move(std::thread(storage_func, _pFilter, _storage_folder)));
}

void CHddStorage::saveFilter(CCycFilterBase* _pFilter, std::ofstream& _csv_writter, const std::string& _storage_dir, std::mutex& m)
{
    if (!m_setup_done.at(_pFilter->getFilterKey()))
    {
        setup_directories(_pFilter, _storage_dir);
        write_header(_pFilter, _csv_writter);
        m_setup_done[_pFilter->getFilterKey()] = true;
    }
    
    switch (_pFilter->getOutputDataType())
    {
        case CyC_CONTROL_INPUT:
        {
            CycControlInput controlInput;
            const bool bDataRead = _pFilter->getData(controlInput);
            if (bDataRead)
            {
                _csv_writter << _pFilter->getTimestampStop() << "," << _pFilter->getSamplingTime();
                for (auto idx = 0; idx < controlInput.u.size(); ++idx)
                {
                    _csv_writter << "," << controlInput.u[idx];
                }
                _csv_writter << std::endl;
            }

        }
        break;
        case CyC_REFERENCE_SETPOINTS:
        {
            CycReferenceSetPoints refSetpoints;
            if (_pFilter->getData(refSetpoints))
            {
                if (m_extra_writers.find(_pFilter->getFilterKey()) == m_extra_writers.end())
                {
                    csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                    m_frame_ids[_pFilter->getFilterKey()] = 0;

                    if (!extra_writer.open(_storage_dir + "/framebased_data_descriptor.csv"))
                    {
                        spdlog::error("Filter[{}-{}]: Failed to open framebased data descriptor csv.", _pFilter->getFilterKey().nCoreID, _pFilter->getFilterKey().nFilterID);
                    }
                    else
                    {
                        std::vector<std::string> columnNames(refSetpoints.ref[0].size() + 1);
                        columnNames[0] = "frame_id";
                        size_t i = 0;
                        std::generate(columnNames.begin() + 1, columnNames.end(), [&i]() { return fmt::format("ref_point_{}", i++);  });
                        extra_writer.set_column_names(columnNames);
                    }
                }


                csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                CyC_INT& frame_id = m_frame_ids[_pFilter->getFilterKey()];

                if (extra_writer.is_open())
                {
                    for (const auto& refPt : refSetpoints.ref)
                    {
                        auto crtRow = extra_writer.new_row();
                        crtRow.write_column(frame_id);
                        for (auto idx = 0; idx < refPt.size(); ++idx)
                        {
                            crtRow.write_column(refPt[idx]);
                        }
                    }

                    _csv_writter << std::fixed << std::setprecision(6)
                        << _pFilter->getTimestampStop() << ','
                        << _pFilter->getSamplingTime() << ','
                        << frame_id << std::endl;
                }
                ++frame_id;
            }


        }
        break;
        case CyC_IMAGE:
        {
            CycImages frames;
            bool bDataRead = _pFilter->getData(frames);

            if (bDataRead)
            {
                const auto _ts_stop = _pFilter->getTimestampStop();
                const auto _sample_time = _pFilter->getSamplingTime();
                const auto _ts_start = _ts_stop - _sample_time;

                m_save_thread_pool.try_enqueue(
                    [this, _ts_start, _ts_stop, _sample_time, &_storage_dir, &_csv_writter, &m](const CycImages& imgs) {
                    std::lock_guard<std::mutex> guard(m);
                    saveCycImage(imgs, _ts_start, _ts_stop, _sample_time, _csv_writter, _storage_dir);
                }, std::move(frames));
            }
        }
        break;

        case CyC_IMU:
        {
            CycImus imu_cache;
            bool bDataRead = _pFilter->getData(imu_cache);

            if (bDataRead)
            {
                std::string sFilePathImu = "samples/" + std::to_string(_pFilter->getTimestampStop()) + ".csv";
                _csv_writter << std::to_string(_pFilter->getTimestampStop()) << "," <<
                    std::to_string(_pFilter->getSamplingTime()) << "," <<
                    sFilePathImu << std::endl;

                std::ofstream csv_writter(_storage_dir + "/" + sFilePathImu);

                // Write header
                csv_writter << "timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z" << std::endl;

                // Write data
                for (const auto& imu : imu_cache)
                {
                    csv_writter << imu.timestamp << "," << imu.acc.x() << "," << imu.acc.y() << "," << imu.acc.z() << "," <<
                        imu.gyro.x() << "," << imu.gyro.y() << "," << imu.gyro.z() << std::endl;
                }
            }
        }
        break;

        case CyC_GPS:
        {
            CycGps gps;
            bool bDataRead = _pFilter->getData(gps);

            if (bDataRead)
            {
                _csv_writter << std::fixed << std::setprecision(6) << 
                    std::to_string(_pFilter->getTimestampStop()) << "," <<
                    std::to_string(_pFilter->getSamplingTime()) << "," <<
                    gps.lat << "," << gps.lng << "," <<
                    std::fixed << std::setprecision(1) << gps.alt << std::endl;
            }
        }
        break;

        case CyC_STATE:
        {
            CycState state;
            bool bDataRead = _pFilter->getData(state);

            if (bDataRead)
            {
                _csv_writter << std::fixed << std::setprecision(6) << 
                    std::to_string(_pFilter->getTimestampStop()) << "," <<
                    std::to_string(_pFilter->getSamplingTime());

                for (CyC_UINT i = 0; i < state.x_hat.size(); ++i)
                    _csv_writter << "," << std::to_string(state.x_hat(i));

                _csv_writter << std::endl;
            }
        }
        break;

        case CyC_2D_ROIS:
        {
            if (m_extra_writers.find(_pFilter->getFilterKey()) == m_extra_writers.end())
            {
                csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                m_frame_ids[_pFilter->getFilterKey()] = 0;

                if (!extra_writer.open(_storage_dir + "/framebased_data_descriptor.csv"))
                {
                    spdlog::error("Filter[{}-{}]: Failed to open framebased data descriptor csv.", _pFilter->getFilterKey().nCoreID, _pFilter->getFilterKey().nFilterID);
                }
                else
                {
                    extra_writer.set_column_names("frame_id", "roi_id", "cls", "x", "y", "width", "height");
                }
            }

            CycRois2D rois;
            if (_pFilter->getData(rois))
            {
                csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                CyC_INT& frame_id = m_frame_ids[_pFilter->getFilterKey()];

                if (extra_writer.is_open())
                {
                    for (const auto& roi : rois)
                    {
                        extra_writer.write_row(frame_id, roi.id, roi.cls, roi.origin.x(), roi.origin.y(), roi.width, roi.height);
                    }

                    _csv_writter << std::fixed << std::setprecision(6)
                        << _pFilter->getTimestampStop() << ','
                        << _pFilter->getSamplingTime() << ','
                        << frame_id << std::endl;
                }

                ++frame_id;
            }
        }

        case CyC_POINTS:
        {
            if (m_extra_writers.find(_pFilter->getFilterKey()) == m_extra_writers.end())
            {
                csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                m_frame_ids[_pFilter->getFilterKey()] = 0;

                if (!extra_writer.open(_storage_dir + "/framebased_data_descriptor.csv"))
                {
                    spdlog::error("Filter[{}-{}]: Failed to open framebased data descriptor csv.", _pFilter->getFilterKey().nCoreID, _pFilter->getFilterKey().nFilterID);
                }
                else
                {
                    extra_writer.set_column_names("frame_id", "x", "y", "id", "score");
                }
            }

            CycPoints pts;
            if (_pFilter->getData(pts))
            {
                csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                CyC_INT& frame_id = m_frame_ids[_pFilter->getFilterKey()];

                if (extra_writer.is_open())
                {
                    for (const auto& pt : pts)
                    {
                        extra_writer.write_row(frame_id, pt.pt2d.x(), pt.pt2d.y(), pt.id, pt.score);
                    }

                    _csv_writter << std::fixed << std::setprecision(6)
                        << _pFilter->getTimestampStop() << ','
                        << _pFilter->getSamplingTime() << ','
                        << frame_id << std::endl;
                }

                ++frame_id;
            }
        }
        break;
        case CyC_VECTOR_INT:
        {
            std::vector<CyC_INT> data;
            bool bDataRead = _pFilter->getData(data);

            if (bDataRead)
            {
                _csv_writter << _pFilter->getTimestampStop() << "," << _pFilter->getSamplingTime();

                for (CyC_UINT i = 0; i < data.size(); ++i)
                    _csv_writter << "," << data[i];

                _csv_writter << std::endl;
            }
        }
        break;

        case CyC_VECTOR_FLOAT:
        {
            std::vector<float> data;
            bool bDataRead = _pFilter->getData(data);

            if (bDataRead)
            {
                _csv_writter << _pFilter->getTimestampStop() << "," << _pFilter->getSamplingTime();

                for (CyC_UINT i = 0; i < data.size(); ++i)
                    _csv_writter << "," << data[i];

                _csv_writter << std::endl;
            }
        }
        break;

        case CyC_OCTREE:
        {
            CycEnvironment data(0.2f);
            bool bDataRead = _pFilter->getData(data);

            if (bDataRead)
            {
                const auto _ts_stop = _pFilter->getTimestampStop();

                std::string _octree_path = "samples/" + std::to_string(_ts_stop) + ".bt";
                _csv_writter << _pFilter->getTimestampStop() << "," << _pFilter->getSamplingTime() << "," << _octree_path << std::endl;

                std::ofstream stream_out(_storage_dir + "/" + _octree_path);
                data.pOccupancyModel->writeData(stream_out);
                stream_out.flush();
            }
        }
        break;

        case CyC_VOXELS:
        {
            CycVoxels voxels;
            bool bDataRead = _pFilter->getData(voxels);

            if (bDataRead)
            {
                const auto _ts_stop = _pFilter->getTimestampStop();

                std::string _voxels_path = "samples/" + std::to_string(_ts_stop) + ".data";
                _csv_writter << _pFilter->getTimestampStop() << "," << _pFilter->getSamplingTime() << "," << _voxels_path << std::endl;

                std::ofstream stream_out(_storage_dir + "/" + _voxels_path, std::ios::binary);

                int64_t voxels_size = voxels.size();

                stream_out.write((char*)&voxels_size, sizeof(voxels_size));

                for (const auto& voxel : voxels)
                {
                    stream_out.write((char*)&voxel.id, sizeof(voxel.id));
                    stream_out.write((char*)&voxel.error, sizeof(voxel.error));
                    stream_out.write((char*)&voxel.pt3d.x(), sizeof(voxel.pt3d.x()));
                    stream_out.write((char*)&voxel.pt3d.y(), sizeof(voxel.pt3d.y()));
                    stream_out.write((char*)&voxel.pt3d.z(), sizeof(voxel.pt3d.z()));
                    stream_out.write((char*)&voxel.pt3d.w(), sizeof(voxel.pt3d.w()));
                }

                stream_out.close();
            }
        }
        break;

        case CyC_POSES_6D:
        {
            if (m_extra_writers.find(_pFilter->getFilterKey()) == m_extra_writers.end())
            {
                csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                m_frame_ids[_pFilter->getFilterKey()] = 0;

                if (!extra_writer.open(_storage_dir + "/framebased_data_descriptor.csv"))
                {
                    spdlog::error("Filter[{}-{}]: Failed to open framebased data descriptor csv.", _pFilter->getFilterKey().nCoreID, _pFilter->getFilterKey().nFilterID);
                }
                else
                {
                    extra_writer.set_column_names("frame_id", "id", "x", "y", "z", "roll", "pitch", "yaw");
                }
            }

            CycPoses poses;
            if (_pFilter->getData(poses))
            {
                csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                CyC_INT& frame_id = m_frame_ids[_pFilter->getFilterKey()];

                if (extra_writer.is_open())
                {
                    for (const auto& pose : poses)
                    {
                        const Eigen::Vector3f t = pose.translation_3x1();
                        const Eigen::Vector3f r = pose.rotation_euler();

                        extra_writer.write_row(frame_id, pose.getID(), t[0], t[1], t[2], r[0], r[1], r[2]);
                    }

                    _csv_writter << std::fixed << std::setprecision(6)
                        << _pFilter->getTimestampStop() << ','
                        << _pFilter->getSamplingTime() << ','
                        << frame_id << std::endl;
                }

                ++frame_id;
            }
        }
        break;

        case CyC_VECTOR_STRING:
        {
            if (m_extra_writers.find(_pFilter->getFilterKey()) == m_extra_writers.end())
            {
                csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                m_frame_ids[_pFilter->getFilterKey()] = 0;

                if (!extra_writer.open(_storage_dir + "/framebased_data_descriptor.csv"))
                {
                    spdlog::error("Filter[{}-{}]: Failed to open framebased data descriptor csv.", _pFilter->getFilterKey().nCoreID, _pFilter->getFilterKey().nFilterID);
                }
                else
                {
                    extra_writer.set_column_names("frame_id", "string");
                }
            }

            std::vector<std::string> strings;
            if (_pFilter->getData(strings))
            {
                csv::writer& extra_writer = m_extra_writers[_pFilter->getFilterKey()];
                CyC_INT& frame_id = m_frame_ids[_pFilter->getFilterKey()];

                if (extra_writer.is_open())
                {
                    for (const auto& str : strings)
                    {
                        extra_writer.write_row(frame_id, str);
                    }

                    _csv_writter << std::fixed << std::setprecision(6)
                        << _pFilter->getTimestampStop() << ','
                        << _pFilter->getSamplingTime() << ','
                        << frame_id << std::endl;
                }

                ++frame_id;
            }
        }
        break;

        case CyC_TERMINAL_DATA:
        {
            CycTerminalCommand term;
            if (_pFilter->getData(term))
            {
                _csv_writter << _pFilter->getTimestampStop() << "," << _pFilter->getSamplingTime() << "," << term.cmd[0] << "," << term.cmd[1] << std::endl;
            }
        }
        break;

        default:
            spdlog::error("CHddStorage: Cannot write data for type: {}, {}", static_cast<int>(_pFilter->getOutputDataType()), _pFilter->getFilterName());
            break;
    }

    _csv_writter.flush();
}

bool CHddStorage::saveCycImage(
    const CycImages& _images,
    const CyC_TIME_UNIT& _timestamp_start,
    const CyC_TIME_UNIT& _timestamp_stop,
    const CyC_TIME_UNIT& _sampling_time,
    std::ofstream& _csv_writer,
    const std::string& _full_path)
{
    bool bReturn = true;

    _csv_writer << _timestamp_stop << ',' << _sampling_time;

    for (size_t i = 0; i < _images.size(); ++i)
    {
        std::string sFilePathImgLeft = "samples/" + std::to_string(i) + "/left/" + std::to_string(_timestamp_stop) + ".png";
        std::string sFilePathImgRight;

        const cv::Mat img_left(_images[i].nRows, _images[i].nCols, _images[i].nType1, _images[i].pData1);
        bReturn &= cv::imwrite(_full_path + "/" + sFilePathImgLeft, img_left);

        if (_images[i].bIsStereo)
        {
            sFilePathImgRight = "samples/" + std::to_string(i) + "/right/" + std::to_string(_timestamp_stop) + ".png";

            const cv::Mat img_right(_images[i].nRows, _images[i].nCols, _images[i].nType2, _images[i].pData2);
            const cv::Mat img_right_out = CDepthImageProcessing::depth2image(img_right);

            bReturn &= cv::imwrite(_full_path + "/" + sFilePathImgRight, img_right_out);
        }

        _csv_writer << "," << _images[i].nTimestamp << "," << sFilePathImgLeft << "," << sFilePathImgRight;
    }

    _csv_writer << std::endl;
    return bReturn;
}

void CHddStorage::checkCcrDBIntegrity(const std::string& _db_folder)
{
    if (!fs::exists(_db_folder))
    {
        spdlog::error("CHddStorage::checkCcrDBIntegrity(): CCR database \'{}\' does not exist.", _db_folder);
    }
    else
    {
        std::string line, colname;
        std::ifstream CsvTimestampsSyncFile(_db_folder + "/sampling_timestamps_sync.csv");

        // Make sure the file is open
        if (!CsvTimestampsSyncFile.is_open())
        {
            spdlog::error("CHddStorage::checkCcrDBIntegrity(): Could not open sampling_timestamps_sync file in DB folder \'{}\'", _db_folder);
            return;
        }

        // Get the number of columns in the sampling_timestamps_sync.csv file
        std::getline(CsvTimestampsSyncFile, line);
        std::stringstream ss(line);

        // Map each column id to the replay filters
        CyC_UINT nColumns = 0;
        std::vector<std::string> vColumnsNames;
        std::vector<std::string> vLastTimestamp;
        while (std::getline(ss, colname, ','))
        {
            ++nColumns;
            vColumnsNames.push_back(colname);
            vLastTimestamp.push_back("-1");
        }
        spdlog::info("CHddStorage::checkCcrDBIntegrity(): {} columns found in DB  \'{}\'", nColumns, _db_folder);

        // Check if two consecutive line contain the same time stamp
        // Read data, line by line
        bool bIntegrityOK(true);
        CyC_UINT idxLine(2);
        while (std::getline(CsvTimestampsSyncFile, line))
        {
            // Create a stringstream from line
            std::stringstream ss(line);
            std::string valCol;

            // Extract each column
            CyC_UINT idxCol(0);
            while (std::getline(ss, valCol, ','))
            {
                if (valCol.compare("-1") != 0)
                {
                    if (valCol.compare(vLastTimestamp[idxCol]) == 0)
                    {
                        bIntegrityOK = false;
                        std::cout << "Repeated timestamp in " << vColumnsNames[idxCol] << ", line " << idxLine << std::endl;
                    }

                    vLastTimestamp[idxCol] = valCol;
                }
                ++idxCol;
            }

            ++idxLine;
        }

        if (bIntegrityOK)
            std::cout << "Integrity check for CCR database \'" << _db_folder << "\' passed." << std::endl;

        // Close the sampling_timestamps_sync.csv file
        CsvTimestampsSyncFile.close();
    }
}
