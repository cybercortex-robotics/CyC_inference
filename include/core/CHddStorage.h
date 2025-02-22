// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CHddStorage_H_
#define CHddStorage_H_

#include "CCR_TYPES.h"
#include "CBaseCcrFilter.h"
#include "core/CCcrCore.h"
#include "os/CTimer.h"
#include "os/ThreadPool.h"
#include "csv_writer.h"

class CHddStorage
{
public:
	CHddStorage(CCcrCore* _core, const std::string& _read_path, const std::string& _save_path);
	virtual ~CHddStorage();

    bool saveDatablockAsync();

    bool readDatablockSynced();
    bool saveDatablockSynced();

    static void checkCcrDBIntegrity(const std::string& _db_folder);

private:
    void generateDatablockDescriptor();
    void generateFilterOutputStructures(CBaseCcrFilter* _pFilter, std::string _datastream_storage_folder);

    void startDataSavingThreads(CBaseCcrFilter* _pFilter, std::string _datastream_storage_folder);
    void saveFilter(CBaseCcrFilter* _pFilter, std::ofstream& _csv_writter, const std::string& _datastream_storage_folder, std::mutex& m);
    
    static bool saveCcrImage(
            const CcrImages& _images,
            const CCR_TIME_UNIT& _timestamp_start,
            const CCR_TIME_UNIT& _timestamp_stop,
            const CCR_TIME_UNIT& _sampling_time,
            std::ofstream& _csv_writer,
            const std::string& _full_path);

private:
    CCcrCore*   m_pCcrCore;
    std::string m_sSaveDBStoragePath;
    std::string m_sReadDBStoragePath;

    bool m_bReadDBEnabled;
    bool m_bSaveReadSamePath;

    std::vector<std::thread>                            m_vSavingThreads;
    std::unordered_map<CcrDatablockKey, std::mutex*>    m_mapSavingMutexes;
    std::unordered_map<CcrDatablockKey, CCR_TIME_UNIT>  m_mapSyncSavingTimestamps;
    std::unordered_map<CcrDatablockKey, bool>           m_mapIsSaving;

    std::thread     m_save_thread;
    CCR_ATOMIC_BOOL m_save_running;
    ThreadPool      m_save_thread_pool;

    std::thread     m_read_thread;
    CCR_ATOMIC_BOOL m_read_running;
    ThreadPool      m_read_thread_pool;

    std::unordered_map<CcrDatablockKey, csv::writer>    m_extra_writers;
    std::unordered_map<CcrDatablockKey, CCR_INT>        m_frame_ids;
    std::unordered_map<CcrDatablockKey, bool>           m_setup_done;
};

#endif /* CHddStorage_H_ */
