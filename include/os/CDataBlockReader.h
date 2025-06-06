// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CDataBlockReader_H
#define CDataBlockReader_H

#include "CyC_TYPES.h"
#include "csv_reader.h"
#include <iostream>
#include <fstream>
#include <cstdarg>
#include <opencv2/opencv.hpp>
#include "os/CStringUtils.h"

class CDataBlockReader
{
public:
    typedef std::variant<
        float,
        std::vector<float>,
        CycImage_,
        CycPoints,
        CycState,
        CycImus
    > DataType;

    struct DatablockData
    {
        CyC_INT         filter_id = -1;
        CyC_TIME_UNIT   timestamp = -1;
        DataType        data;

        DatablockData(const CyC_INT _filter_id) :
            filter_id(_filter_id)
        {}
    };

private:
    struct Datastream
    {
        CyC_INT         filter_id = -1;
        CyC_INT         datatype = -1;
        std::string     imgs_datastream_name;
        std::ifstream   imgs_datastream_reader;
        CyC_INT         imgs_col = -1;
    };
    typedef std::vector<Datastream> Datastreams;

public:
	CDataBlockReader(const std::string _db_path, const std::vector<CyC_INT> _filter_ids);
    ~CDataBlockReader();

    bool getNextRow(std::vector<DatablockData>& _out_data);

    bool readImage(const std::string& _line, const std::string& _datastream_name, DataType& _out_img);
    bool readKeypts(const std::string& _line, const std::string& _datastream_name, DataType& _out_keypts);
    bool readState(const std::string& _line, const std::string& _datastream_name, DataType& _out_state);
    bool readImu(const std::string& _line, const std::string& _datastream_name, DataType& _out_imu);

private:
    bool makeDatastream(const std::string& _db_path, const CyC_INT& _filter_id, const CyC_INT& _datatype, CDataBlockReader::Datastream& _out_datastream);

private:
    bool        m_bInitialized = false;
    std::string m_DatabasePath;
    csv::reader m_SyncedCsvReader;
    Datastreams m_Datastreams;
};

#endif //CDataBlockReader_H
