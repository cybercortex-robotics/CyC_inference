// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CHddStorage_H_
#define CHddStorage_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include "CCycCore.h"
#include "os/CTimer.h"
#include "os/ThreadPool.h"
#include "os/CCsvWriter.h"

class CHddStorage
{
public:
	CHddStorage(CCycCore* _core, const std::string& _read_path, const std::string& _save_path);
	virtual ~CHddStorage();

    bool saveDatablockAsync();

    bool readDatablockSynced();
    bool saveDatablockSynced();

    // True once the replay has sent the last sample of the datablock to its filters (and they have loaded it).
    // Stays false when there is no replay, or when the replay was stopped before its end.
    bool isReplayFinished() const { return m_read_finished; }

    // Stops the replay thread and waits for the samples it has already queued
    void stopReplay();

    static void checkCcrDBIntegrity(const std::string& _db_folder);

private:
    void generateDatablockDescriptor();
    void generateFilterOutputStructures(CCycFilterBase* _pFilter, std::string _datastream_storage_folder);

    void startDataSavingThreads(CCycFilterBase* _pFilter, std::string _datastream_storage_folder);
    void saveFilter(CCycFilterBase* _pFilter, std::ofstream& _csv_writter, const std::string& _datastream_storage_folder, std::mutex& m);
    
    static bool saveCycImage(
            const CycImages& _images,
            const CyC_TIME_UNIT& _timestamp_start,
            const CyC_TIME_UNIT& _timestamp_stop,
            const CyC_TIME_UNIT& _sampling_time,
            std::ofstream& _csv_writer,
            const std::string& _full_path);

private:
    CCycCore*   m_pCycCore;
    std::string m_sSaveDBStoragePath;
    std::string m_sReadDBStoragePath;

    bool m_bReadDBEnabled;
    bool m_bSaveReadSamePath;

    std::vector<std::thread>                            m_vSavingThreads;
    std::unordered_map<CycDatablockKey, std::mutex*>    m_mapSavingMutexes;
    std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>  m_mapSyncSavingTimestamps;
    std::unordered_map<CycDatablockKey, bool>           m_mapIsSaving;

    std::thread     m_save_thread;
    CyC_ATOMIC_BOOL m_save_running;
    ThreadPool      m_save_thread_pool;

    std::thread     m_read_thread;
    CyC_ATOMIC_BOOL m_read_running;
    CyC_ATOMIC_BOOL m_read_finished{false};
    std::atomic<int> m_read_pending{0};     // samples queued in m_read_thread_pool, not yet loaded by their filter
    ThreadPool      m_read_thread_pool;

    std::unordered_map<CycDatablockKey, csv::writer>    m_extra_writers;
    std::unordered_map<CycDatablockKey, CyC_INT>        m_frame_ids;
    std::unordered_map<CycDatablockKey, bool>           m_setup_done;
};

#endif /* CHddStorage_H_ */
