// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CDataBlockReader.h"

CDataBlockReader::CDataBlockReader(const std::string _db_path, const std::vector<CyC_INT> _filter_ids) :
    m_DatabasePath(_db_path)
{
    m_bInitialized = false;

    // Check if the database exists
    if (!m_SyncedCsvReader.open(m_DatabasePath + "/sampling_timestamps_sync.csv"))
    {
        spdlog::error("{}: Failed to read synced timestamps csv.", typeid(*this).name());
        return;
    }

    // Check if the filter IDs exist in the database
    std::vector<std::string> column_names = m_SyncedCsvReader.get_column_names();
    std::vector<CyC_INT> available_datastreams;
    for (size_t i = 1; i < column_names.size(); ++i)
    {
        const std::string col_name = column_names[i];
        std::vector<std::string> split;
        CStringUtils::splitstring(col_name, "_", split);

        if (split.size() != 2)
        {
            spdlog::error("{}: Bad datastream name format '{}'.", typeid(*this).name(), col_name);
            continue;
        }

        if (!CStringUtils::is_positive_int(split[1]))
        {
            spdlog::error("{}: Bad datastream number format '{}'.", typeid(*this).name(), split[1]);
            continue;
        }

        available_datastreams.emplace_back(std::stoi(split[1]));
    }

    for (const CyC_INT& filter_id : _filter_ids)
    {
        bool bDsFound = false;
        for (const CyC_INT& ds_id : available_datastreams)
        {
            if (filter_id == ds_id)
            {
                bDsFound = true;
                break;
            }
        }

        if (!bDsFound)
        {
            spdlog::error("{}: Datastream for filter ID '{}' not found in the given DataBlock.", typeid(*this).name(), filter_id);
            return;
        }
    }

    // Get the filters datatypes
    csv::reader DataBlockDescriptorCsv;
    if (!DataBlockDescriptorCsv.open(m_DatabasePath + "/datablock_descriptor.csv"))
    {
        spdlog::error("{}: Failed the datablock descriptor csv.", typeid(*this).name());
        return;
    }
    const csv::reader::row& datablock_descriptor_row = DataBlockDescriptorCsv.get_row();
    while (DataBlockDescriptorCsv.next_row())
    {
        for (const CyC_INT& ds_id : _filter_ids)
        {
            if (ds_id == datablock_descriptor_row.get<CyC_INT>(1))
            {
                Datastream datastream;
                CyC_INT datatype = datablock_descriptor_row.get<CyC_INT>(4);
                makeDatastream(m_DatabasePath, ds_id, datatype, datastream);
                m_Datastreams.emplace_back(std::move(datastream));
            }
        }
    }

    m_bInitialized = true;
}

CDataBlockReader::~CDataBlockReader()
{}

bool CDataBlockReader::getNextRow(std::vector<DatablockData>& _out_data)
{
    // Check if the reader is initialized
    if (!m_bInitialized)
    {
        spdlog::error("{}: DataBlock reader not initialized. Check initialization errors.", typeid(*this).name());
        return false;
    }

    // Check if the size of the requested data vector is equal to the read filters
    if (_out_data.size() != m_Datastreams.size())
    {
        spdlog::error("{}: Requested data size is different from the number of parsed filters.", typeid(*this).name());
        return false;
    }

    // Check if the next timestamps row is available
    if(!m_SyncedCsvReader.next_row())
    {
        spdlog::error("{}: DataBlock timestamps sync file ended.", typeid(*this).name());
        return false;
    }

    // Set datablock reading timestamps tp -1
    for (DatablockData& data_blk : _out_data)
        data_blk.timestamp = -1;

    const csv::reader::row& row = m_SyncedCsvReader.get_row();

    for (size_t k = 1; k < row.size(); ++k)
    {
        const CyC_TIME_UNIT filter_timestamp = row.get<CyC_TIME_UNIT>(k);

        if (filter_timestamp != CyC_TIME_UNIT(-1))
        {
            for (Datastream& ds : m_Datastreams)
            {
                if (k == ds.imgs_col)
                {
                    for (DatablockData& data_blk : _out_data)
                    {
                        if (data_blk.filter_id == ds.filter_id)
                        {
                            data_blk.timestamp = filter_timestamp;

                            std::string line;
                            if (std::getline(ds.imgs_datastream_reader, line))
                            {
                                switch (ds.datatype)
                                {
                                    case CyC_IMAGE:
                                        readImage(line, ds.imgs_datastream_name, data_blk.data);
                                        break;
                                    case CyC_POINTS:
                                        readKeypts(line, ds.imgs_datastream_name, data_blk.data);
                                        break;
                                    case CyC_STATE:
                                        readState(line, ds.imgs_datastream_name, data_blk.data);
                                        break;
                                    case CyC_IMU:
                                        readImu(line, ds.imgs_datastream_name, data_blk.data);
                                        break;
                                    default:
                                        break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return true;
}

bool CDataBlockReader::makeDatastream(const std::string& _db_path, const CyC_INT& _filter_id, const CyC_INT& _datatype, CDataBlockReader::Datastream& _out_datastream)
{
    _out_datastream.filter_id = _filter_id;
    _out_datastream.datatype = _datatype;
    _out_datastream.imgs_datastream_name = fmt::format("datastream_{}", _filter_id);
    const std::string imgs_descriptor_path = _db_path + "/" + _out_datastream.imgs_datastream_name + "/data_descriptor.csv";
    _out_datastream.imgs_datastream_reader.open(imgs_descriptor_path);
    
    std::string line;
    if (!_out_datastream.imgs_datastream_reader.is_open() || !std::getline(_out_datastream.imgs_datastream_reader, line))
    {
        spdlog::error("{}: Failed to read datastream {}.", typeid(*this).name(), _filter_id);
    }
    
    const std::vector<std::string>::const_iterator imgs_col_iterator = std::find(
        m_SyncedCsvReader.get_column_names().begin(),
        m_SyncedCsvReader.get_column_names().end(),
        _out_datastream.imgs_datastream_name);
    
    if (imgs_col_iterator == m_SyncedCsvReader.get_column_names().end())
    {
        spdlog::error("{}: Datastream for filter ID {} could not be bound.", typeid(*this).name(), _filter_id);
        return false;
    }
    else
    {
        _out_datastream.imgs_col = static_cast<CyC_INT>(std::distance(m_SyncedCsvReader.get_column_names().begin(), imgs_col_iterator));
        return true;
    }
}

bool CDataBlockReader::readImage(const std::string& _line, const std::string& _datastream_name, DataType& _out_img)
{
    csv::reader::row row;
    row.parse_line(_line, ',');

    enum { TS_STOP, SAMPLING_TIME, TS_IMAGE, IMAGE_PATH_LEFT, IMAGE_PATH_RIGHT, NUM };
    if (row.size() != NUM)
    {
        spdlog::error("{}: Images reading: wrong number of columns. {} provided, but expected {}.", typeid(*this).name(), row.size(), static_cast<int>(NUM));
        return false;
    }

    CyC_TIME_UNIT ts_image = row.get<CyC_TIME_UNIT>(TS_IMAGE);
    std::string img_path = m_DatabasePath + "/" + _datastream_name + "/" + row.get<std::string>(IMAGE_PATH_LEFT);
    cv::Mat img_left = cv::imread(img_path, cv::IMREAD_ANYCOLOR);

    if (row.get<std::string>(IMAGE_PATH_RIGHT).size() == 0)
    {
        _out_img = CycImage_(img_left, cv::Mat(), ts_image);
    }
    else
    {
        img_path = m_DatabasePath + "/" + _datastream_name + "/" + row.get<std::string>(IMAGE_PATH_RIGHT);
        cv::Mat img_right = cv::imread(img_path, cv::IMREAD_ANYCOLOR);
        _out_img = CycImage_(img_left, img_right, ts_image);
    }

    return true;
}

bool CDataBlockReader::readKeypts(const std::string& _line, const std::string& _datastream_name, DataType& _out_keypts)
{
    csv::reader::row row;
    row.parse_line(_line, ',');

    enum { TS_STOP, SAMPLING_TIME, FRAME_ID, NUM };
    if (row.size() != NUM)
    {
        spdlog::error("{}: Keypoints reading: wrong number of columns. {} provided, but expected {}.", typeid(*this).name(), row.size(), static_cast<int>(NUM));
        return false;
    }

    const CyC_INT sync_frame_id = row.get<CyC_INT>(FRAME_ID);

    csv::reader framebased_csv_reader;
    if (!framebased_csv_reader.open(m_DatabasePath + "/" + _datastream_name + "/framebased_data_descriptor.csv"))
    {
        spdlog::error("{}: Failed to read framebased_csv_reader.", typeid(*this).name());
        return false;
    }
    else
    {
        CycPoints pts;
        while (framebased_csv_reader.next_row())
        {
            const csv::reader::row& row_pt = framebased_csv_reader.get_row();

            enum { PT_FRAME_ID, PT_X, PT_Y, PT_ID, PT_SCORE, PT_NUM };

            if (row_pt.get<CyC_INT>(PT_FRAME_ID) == sync_frame_id)
            {
                CycPoint pt;
                pt.id = row_pt.get<CyC_INT>(PT_ID);
                pt.score = static_cast<float>(row_pt.get<CyC_INT>(PT_SCORE));
                pt.pt2d.x() = static_cast<float>(row_pt.get<CyC_INT>(PT_X));
                pt.pt2d.y() = static_cast<float>(row_pt.get<CyC_INT>(PT_Y));
                pts.emplace_back(pt);
            }
        }

        _out_keypts = pts;
    }

    return true;
}

bool CDataBlockReader::readState(const std::string& _line, const std::string& _datastream_name, DataType& _out_state)
{
    CycState state;

    csv::reader::row row;
    row.parse_line(_line, ',');

    enum { TS_STOP, SAMPLING_TIME, NUM };
    if (row.size() <= NUM)
    {
        spdlog::error("{}: State reading: wrong number of columns. {} provided, but expected {} or more.", typeid(*this).name(), row.size(), NUM + 1);
        return false;
    }
    else
    {
        state.x_hat.resize(row.size() - NUM);
        for (size_t k = NUM; k < row.size(); ++k)
        {
            state.x_hat[k - NUM] = row.get<float>(k);
        }
    }

    _out_state = state;
    return true;
}

bool CDataBlockReader::readImu(const std::string& _line, const std::string& _datastream_name, DataType& _out_imu)
{
    CycImus imu_cache;

    csv::reader::row row;
    row.parse_line(_line, ',');
    enum { TS_STOP, SAMPLING_TIME, IMU_PATH, NUM };
    if (row.size() < NUM)
    {
        spdlog::error("{}: Wrong number of columns. {} provided, but expected at least {}.", typeid(*this).name(), row.size(), NUM + 1);
        return false;
    }

    if (row.get<std::string>(IMU_PATH).size() != 0)
    {
        std::string imu_path = m_DatabasePath + "/" + _datastream_name + "/" + row.get<std::string>(IMU_PATH);
        csv::reader imu_reader;
        if (!imu_reader.open(imu_path))
        {
            spdlog::error("{}: Failed to open imu cache path '{}'.", typeid(*this).name(), imu_path);
        }
        else
        {
            const csv::reader::row& imu_row = imu_reader.get_row();
            while (imu_reader.next_row())
            {
                CycImu imu;
                imu.timestamp = imu_row.get<CyC_TIME_UNIT>(0);
                imu.acc.x() = imu_row.get<float>(1);
                imu.acc.y() = imu_row.get<float>(2);
                imu.acc.z() = imu_row.get<float>(3);
                imu.gyro.x() = imu_row.get<float>(4);
                imu.gyro.y() = imu_row.get<float>(5);
                imu.gyro.z() = imu_row.get<float>(6);
                imu_cache.emplace_back(imu);
            }
        }
    }

    _out_imu = imu_cache;
    return true;
}
