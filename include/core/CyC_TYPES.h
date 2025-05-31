// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CyC_TYPES_H_
#define CyC_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>
#include <math.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <ctime>
#include <unordered_set>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>

// If compiler >= C++ 17, then use the std::variant implementation, otherwise use the mpark implementation stored in cyc include
#include <variant>

// Environment model types
#include <octomap/octomap.h>
#include <octomap/CcrOcTree.h>

// Coordinates transformation class
#include "env/CPose.h"

// OpenCV include
#include <opencv2/core.hpp>

// Logging library spdlog
#include <spdlog/spdlog-inl.h>

#define CACHE_SIZE	10 // Size of the data cache for each filter

#define EULER		2.71828183f
#define PI			3.14159265358979f   // 180 (PI)
#define PI_2        1.57079632679489f   //  90 (PI / 2)
#define TWO_PI		6.28318530717958f   // 360 (2 * PI)
#define DEG2RAD	    0.01745329251994f   // degree to radian (PI / 180)
#define RAD2DEG     57.2957795130824f   // radian to degree (180 / PI)
#define MPS2KMH     3.6f                // meters per second to Kmh and vice-versa
#define KMH2MPS     0.277777f           // Kmh to meters per second (1 / 3.6)
#define SEC2MSEC    1000.f              // seconds to milliseconds
#define MSEC2SEC    0.001f              // milliseconds to seconds (1 / 1000)
#define GRAVITY     9.81f               // earth's gravitational acceleration is approximately 9.81 m/s^2

typedef uint32_t                        CyC_UINT;
typedef int32_t                         CyC_INT;
typedef uint64_t                        CyC_ULONG;
typedef int64_t                         CyC_LONG;
typedef std::chrono::milliseconds::rep  CyC_TIME_UNIT;
typedef std::atomic<bool>               CyC_ATOMIC_BOOL;
typedef std::atomic<int>                CyC_ATOMIC_INT;
typedef std::atomic<double>             CyC_ATOMIC_DOUBLE;
typedef std::atomic<CyC_TIME_UNIT>      CyC_ATOMIC_TIME_UNIT;

// Data types
enum CyC_DATA_TYPE
{
    // Base types
    CyC_UNDEFINED			    = -1,
    CyC_RAW_DATA_STREAM	        = 0,
    CyC_VECTOR_BOOL             = 28,
    CyC_VECTOR_INT              = 1,
    CyC_VECTOR_FLOAT            = 2,
    CyC_VECTOR_DOUBLE           = 3,
    CyC_VECTOR_STRING           = 25,

    // Sensor data types
    CyC_IMAGE                   = 4,
    CyC_RADAR                   = 5,
    CyC_ULTRASONICS             = 6,
    CyC_IMU                     = 7,
    CyC_GPS                     = 8,

    // Perception types
    CyC_POINTS                  = 9,
    CyC_VOXELS                  = 10,
    CyC_POSES_6D                = 11,
    CyC_2D_ROIS                 = 12,
    CyC_3D_BBOXES               = 13,
    CyC_POINT_FEATURES          = 14,
    CyC_OCTREE                  = 15,
    CyC_GRIDMAP                 = 16,
    CyC_LANES_MODEL             = 17,
    CyC_SLAM                    = 23,

    // Deep learning types
    CyC_DNN_OUTPUT              = 18,

    // Control types
    CyC_REFERENCE_SETPOINTS     = 19,
    CyC_STATE                   = 20,
    CyC_CONTROL_INPUT           = 21,
    CyC_MEASUREMENT             = 22,
    CyC_ESTIMATED_TRAJECTORY    = 24,
    CyC_LANDMARKS               = 26,

    // Communication types
    CyC_TERMINAL_DATA           = 27,
};

// CyC type used for filters (use CStringUtils::CyC_HashFunc for hashing)
using CyC_FILTER_TYPE = size_t;

/*
 * Datablock key used for storing entries in the Datablock
 * An entry is defined by the VisionCore's ID and the Filter's ID
 * The Datablock key must be unique
 */
struct CycDatablockKey
{
    CyC_ULONG   nCoreID;
    CyC_INT     nFilterID;
    std::string sDescription;

    CycDatablockKey() :
        nCoreID(0),
        nFilterID(-1)
    {}

    CycDatablockKey(CyC_INT _core_id, CyC_INT _filter_id, std::string _description = "") :
        nCoreID(_core_id),
        nFilterID(_filter_id),
        sDescription(_description)
    {}

    bool operator==(const CycDatablockKey& key) const
    {
        return (nCoreID == key.nCoreID && nFilterID == key.nFilterID);
    }
};
typedef std::vector<CycDatablockKey> CycDatablockKeys;

// Hash key for undordered maps based on CycDatablockKey
template <>
struct std::hash<CycDatablockKey>
{
    std::size_t operator()(const CycDatablockKey& k) const
    {
        using std::size_t;
        using std::hash;

        // Compute individual hash values for nCoreID and nFilterID and combine them using XOR and bit shifting
        return ((std::hash<CyC_ULONG>()(k.nCoreID) ^ (std::hash<CyC_INT>()(k.nFilterID) << 1)) >> 1);
    }
};

/*
 * Control types
 */
struct CycReferenceSetPoints
{
    // Reference vector calculated via planning (control reference)
    std::vector<Eigen::VectorXf> ref;

    // Samples of reference state trajectories (used mainly for visualization)
    std::vector<std::vector<Eigen::VectorXf>> ref_samples;

    // unique ID for the reference trajectory
    CyC_INT id;
};

struct CycLandmark
{
    CPose                           pose;
    float                           travel_time = 0.f;
    std::vector<Eigen::Vector4f>    waypoints;
};
typedef std::unordered_map<CyC_INT, CycLandmark> CycLandmarks;

struct CycTerminalCommand
{
    std::vector<CyC_INT>    cmd;
    CycLandmarks            landmarks;
    CPose                   destination;
};

struct CycState
{
    CycState()
    {}

    CycState(const CyC_UINT _num_state_variables)
    {
        this->x_hat = Eigen::VectorXf::Zero(_num_state_variables);
    }

    friend auto operator<<(std::ostream& _os, CycState const& _state) -> std::ostream&
    {
        std::string str;
        for (CyC_INT i = 0; i < _state.x_hat.size() - 1; ++i)
            str.append(std::to_string(_state.x_hat[i]) + "\t");

        if (_state.x_hat.size() > 0)
            str.append(std::to_string(_state.x_hat[_state.x_hat.size() - 1]));

        return _os << "State: [" << str << "]";
    }

    Eigen::VectorXf x_hat; // State vector
};

struct CycMeasurement
{
    CycMeasurement()
    {}

    CycMeasurement(const CyC_UINT _num_measurement_variables)
    {
        this->y_hat = Eigen::VectorXf::Zero(_num_measurement_variables);
    }

    Eigen::VectorXf y_hat; // Measurement vector (output)
};

struct CycControlInput
{
    CycControlInput()
    {}

    CycControlInput(const CyC_UINT _num_control_inputs)
    {
        this->u = Eigen::VectorXf::Zero(_num_control_inputs);
    }
    // Control input calculated for setpoint 
    Eigen::VectorXf                 u; // u = [thrust, roll_torque, pitch_torque, yaw_torque]
    CycReferenceSetPoints           ref_pts;
    std::vector<Eigen::VectorXf>    goal_points;
};

/*
 * Sensors data types
 */
 // --- 6D poses vector ---
typedef std::vector<CPose> CycPoses;

// --- IMU ---
struct CycImu
{
    CycImu()
    {
        acc = Eigen::Vector3f{ 0.f, 0.f, 0.f };
        gyro = Eigen::Vector3f{ 0.f, 0.f, 0.f };
        magnet = Eigen::Vector3f{ 0.f, 0.f, 0.f };
        this->timestamp = -1;
    }

    CycImu(const Eigen::Vector3f& _acc, const Eigen::Vector3f& _gyro, const CyC_TIME_UNIT& _timestamp = -1) :
        acc(_acc), gyro(_gyro), timestamp(_timestamp)
    {
        magnet = Eigen::Vector3f{ 0.f, 0.f, 0.f };
    }

    CycImu(const Eigen::Vector3f& _acc, const Eigen::Vector3f& _gyro, const Eigen::Vector3f& _magnet, const CyC_TIME_UNIT& _timestamp = -1) :
        acc(_acc), gyro(_gyro), magnet(_magnet), timestamp(_timestamp)
    {}

    friend auto operator<<(std::ostream& _os, CycImu const& _imu) -> std::ostream&
    {
        return _os << _imu.timestamp << ":\t" << _imu.acc.x() << "\t" << _imu.acc.y() << "\t" << _imu.acc.z() << "\t(" << _imu.gyro.x() << "\t" << _imu.gyro.y() << "\t" << _imu.gyro.z() << ")";
    }

    CyC_TIME_UNIT   timestamp;
    Eigen::Vector3f acc;    // [m/s^2]
    Eigen::Vector3f gyro;   // [deg/s]
    Eigen::Vector3f magnet; // [uT]
};
typedef std::vector<CycImu> CycImus;

// --- Image ---
struct CycImage_
{
    CyC_TIME_UNIT   nTimestamp;     // Image acquisition timestamp
    CyC_UINT	    nRows;		    // Number of image rows
    CyC_UINT	    nCols;		    // Number of image columns
    CyC_UINT	    nChannels1;     // Number of image channels for the first image
    CyC_UINT	    nChannels2;     // Number of image channels for the second image
    CyC_UINT	    nType1;		    // Image type, according to the OpenCV types
	CyC_UINT	    nType2;		    // Image type, for second image, according to the OpenCV types
    CyC_UINT	    nTotalSize1;    // Total image size
	CyC_UINT	    nTotalSize2;    // Total image size for second image
    bool	        bIsStereo;	    // True for stereo or RGB-D images
    void*		    pData1;		    // Pointer to the RGB image data, or left RGB image in case of a stereo image
    void*		    pData2;		    // Pointer to the right RGB image in case of a stereo image, or depth image in case of RGB-D images
    CycDatablockKey key;            // Key of the image source filter

	CycImage_() : 
        nTimestamp(-1),
        nRows(0),
        nCols(0),
        nChannels1(0),
        nChannels2(0),
        nType1(0),
        nType2(0),
        nTotalSize1(0),
        nTotalSize2(0),
        bIsStereo(false),
        pData1(nullptr),
        pData2(nullptr)
    {}

    explicit CycImage_(const cv::Mat& _rgb, const CyC_TIME_UNIT& _ts = -1) :
        nTimestamp(_ts),
        nChannels2(0),
        nType2(0),
        nTotalSize2(0),
        bIsStereo(false),
        pData2(nullptr)
    {
        nRows = _rgb.rows;
        nCols = _rgb.cols;
        nChannels1 = _rgb.channels();
        nType1 = _rgb.type();
        nTotalSize1 = (CyC_UINT)(_rgb.total() * _rgb.elemSize());

        pData1 = malloc(nTotalSize1);
        memcpy(pData1, _rgb.data, nTotalSize1);
    }

    explicit CycImage_(const cv::Mat& _rgb, const cv::Mat& _depth, const CyC_TIME_UNIT& _ts = -1) :
        CycImage_(_rgb, _ts)
    {
        bIsStereo = true;

        nChannels2 = _depth.channels();
        nType2 = _depth.type();
        nTotalSize2 = (CyC_UINT)(_depth.total() * _depth.elemSize());

        pData2 = malloc(nTotalSize2);
        memcpy(pData2, _depth.data, nTotalSize2);
    }

	CycImage_(const CycImage_& _other):
        nTimestamp(_other.nTimestamp),
		bIsStereo(_other.bIsStereo),
		nCols(_other.nCols),
		nRows(_other.nRows),
        nChannels1(_other.nChannels1),
        nChannels2(_other.nChannels2),
		nTotalSize1(_other.nTotalSize1),
		nTotalSize2(_other.nTotalSize2),
		nType1(_other.nType1),
		nType2(_other.nType2),
		pData1(nullptr),
		pData2(nullptr),
        key(_other.key)
	{
		if (_other.pData1 != nullptr)
		{
			pData1 = malloc(_other.nTotalSize1);
			memcpy(pData1, _other.pData1, _other.nTotalSize1);
		}

		if (_other.pData2 != nullptr)
		{
			pData2 = malloc(_other.nTotalSize2);
			memcpy(pData2, _other.pData2, _other.nTotalSize2);
		}
	}

	CycImage_(CycImage_&& _other) noexcept:
        nTimestamp(_other.nTimestamp),
		bIsStereo(_other.bIsStereo),
        nCols(_other.nCols),
		nRows(_other.nRows),
        nChannels1(_other.nChannels1),
        nChannels2(_other.nChannels2),
		nTotalSize1(_other.nTotalSize1),
		nTotalSize2(_other.nTotalSize2),
		nType1(_other.nType1),
		nType2(_other.nType2),
		pData1(_other.pData1),
		pData2(_other.pData2),
        key(_other.key)
	{
        _other.pData1 = nullptr;
        _other.pData2 = nullptr;
	}

	CycImage_& operator=(CycImage_&& _other) noexcept
    {
		if (this != &_other)
        {
            nTimestamp = _other.nTimestamp;
            bIsStereo = _other.bIsStereo;
			nCols = _other.nCols;
			nRows = _other.nRows;
            nChannels1 = _other.nChannels1;
            nChannels2 = _other.nChannels2;
			nTotalSize1 = _other.nTotalSize1;
			nTotalSize2 = _other.nTotalSize2;
			nType1 = _other.nType1;
			nType2 = _other.nType2;

			pData1 = _other.pData1;
			pData2 = _other.pData2;

            _other.pData1 = nullptr;
            _other.pData2 = nullptr;

            key = _other.key;
		}

		return *this;
	}

	CycImage_& operator=(const CycImage_& _other)
    {
		if (this != &_other)
        {
            nTimestamp = _other.nTimestamp;
            bIsStereo = _other.bIsStereo;
            nCols = _other.nCols;
			nRows = _other.nRows;
            nChannels1 = _other.nChannels1;
            nChannels2 = _other.nChannels2;
			nTotalSize1 = _other.nTotalSize1;
			nTotalSize2 = _other.nTotalSize2;
			nType1 = _other.nType1;
			nType2 = _other.nType2;
            key = _other.key;
			
			if (pData1 != nullptr)
            {
			    free(pData1);
                pData1 = nullptr;
            }

			if (pData2 != nullptr)
            {
			    free(pData2);
                pData2 = nullptr;
            }

			if (_other.pData1 != nullptr)
			{
				pData1 = malloc(_other.nTotalSize1);
				memcpy(pData1, _other.pData1, _other.nTotalSize1);
			}

			if (_other.pData2 != nullptr)
			{
				pData2 = malloc(_other.nTotalSize2);
				memcpy(pData2, _other.pData2, _other.nTotalSize2);
			}
		}

		return *this;
	}

    ~CycImage_()
    {
        if (pData1 != nullptr)
        {
			free(pData1);		 
			pData1 = nullptr;
        }

        if (pData2 != nullptr)
        {
			free(pData2);
			pData2 = nullptr;
        }
    }

    bool empty1() const
    {
        return pData1 == 0 || nRows * nCols == 0 || nChannels1 == 0;
    }
    bool empty2() const
    {
        return pData2 == 0 || nRows * nCols == 0 || nChannels2 == 0;
    }
};
typedef std::vector<CycImage_> CycImages;

// --- 2D image point ---
struct CycPoint
{
    CycPoint() :
        id(-1), score(-1.f), depth(-1.f), angle(0.f)
    {
        pt2d = Eigen::Vector2f{ -1.f, -1.f };
        key = CycDatablockKey{ -1, -1 };
    }

    CycPoint(Eigen::Vector2f _pt2D, float _depth = -1.f, CyC_INT _id = -1, float _score = -1.f, cv::Mat _descriptor = cv::Mat(), float _angle = 0.f, CycDatablockKey _key = CycDatablockKey{ -1, -1 }) :
        pt2d(_pt2D), depth(_depth), id(_id), score(_score), key(_key), descriptor(_descriptor), angle(_angle)
    {}
	
	CycPoint(float _x, float _y, float _depth = -1.f, CyC_INT _id = -1, float _score = -1.f, cv::Mat _descriptor = cv::Mat(), float _angle = 0.f, CycDatablockKey _key = CycDatablockKey{ -1, -1 }) :
        depth(_depth), id(_id), score(_score), key(_key), descriptor(_descriptor), angle(_angle)
    {
		pt2d = Eigen::Vector2f {_x, _y };
	}

    friend auto operator<<(std::ostream& _os, CycPoint const& _pt) -> std::ostream&
    {
        return _os << _pt.id << ":\t" << _pt.pt2d.x() << "\t" << _pt.pt2d.y() << "\t" << _pt.depth << "\t(score: " << _pt.score << ", angle: " << _pt.angle << ")";
    }
	
	Eigen::Vector2f pt2d;
    float           depth;
	CyC_INT		    id;
    float           score;

    cv::Mat         descriptor;
    float           angle;

    CycDatablockKey key;  // Key of the image source filter
};
typedef std::vector<CycPoint> CycPoints;

// --- Voxels (point cloud) ---
struct CycVoxel
{
    CycVoxel() :
        id(-1)
    {
        pt3d = Eigen::Vector4f{ 0.f, 0.f, 0.f, 0.f };
    }

    CycVoxel(Eigen::Vector4f _pt3D, CyC_INT _id = -1, float _score = -1.f, float _angle = 0.f, float _error = 9999.f) :
        pt3d(_pt3D), id(_id), error(_error)
    {}

    CycVoxel(float _x, float _y, float _z, float _w = 1.f, CyC_INT _id = -1, float _error = 9999.f) :
        id(_id), error(_error)
    {
        pt3d = Eigen::Vector4f{ _x, _y, _z, _w };
    }

    friend auto operator<<(std::ostream& _os, CycVoxel const& _vx) -> std::ostream&
    {
        return _os << _vx.id << ":\t" << _vx.pt3d.x() << "\t" << _vx.pt3d.y() << "\t" << _vx.pt3d.z() << "\t(err: " << _vx.error << ")";
    }

    Eigen::Vector4f pt3d;
    CyC_INT		    id;
    float           error;
};
typedef std::vector<CycVoxel> CycVoxels;

// --- Ultrasonics ---
struct CycUltrasonic
{
    float   range;
    CPose   pose;
	float   max_range;
};
typedef std::vector<CycUltrasonic> CycUltrasonics;

// --- GPS ---
struct CycGps
{
    float baseLat;
    float baseLng;
    float baseAlt;
    float lat;
    float lng;
    float alt;
};

// TBD: use CycVoxel instead
struct CycLidarPoint
{
    float angle;
    float distance;
    float quality;
};
typedef std::vector<CycLidarPoint> CycLidarPoints;

/*
 * Perception types
 */
struct CycRoi2D
{
    CycRoi2D(Eigen::Vector2f _origin, float _width, float _height, float _conf = 0.F, CyC_INT _cls = -1, CyC_INT _id = -1)
    {
        this->cls = _cls; 
        this->id = _id;
        this->confidence = _conf;
        this->origin = std::move(_origin);
        this->width = _width;
        this->height = _height;
    }

    CycRoi2D()
    {
        this->cls = -1; 
        this->id = -1;
        this->confidence = 0.F;
        this->origin = Eigen::Vector2f(0, 0);
        this->width = 0;
        this->height = 0;
    }

    friend auto operator<<(std::ostream& _os, CycRoi2D const& _roi) -> std::ostream&
    {
        return _os << _roi.id << "\t[" << _roi.key.nCoreID << ", " << _roi.key.nFilterID << "]:\t" << _roi.origin.x() << "\t" << _roi.origin.y() << "\t" << _roi.width << "\t" << _roi.height << "\t(" << _roi.cls << ", " << _roi.confidence << ")";
    }

    // Top left corner is considered as origin
    CyC_INT         id;
    CyC_INT         cls;
    float       confidence;
    Eigen::Vector2f origin;
    float       width;
    float       height;

    // Key of the image source filter for which the ROI was computed
    CycDatablockKey key;
};
typedef std::vector<CycRoi2D> CycRois2D;

struct CycBBox3D
{
	CycBBox3D(CPose _origin, float _width, float _height, float _depth,  CyC_INT _cls = -1, CyC_INT _id = -1)
	{
        this->id = _id;
		this->origin = std::move(_origin);
		this->width = _width;
		this->height = _height;
		this->depth = _depth;
		this->cls = _cls;
	}

	CycBBox3D()
    {
        this->id = -1;
        this->width = 0.f;
        this->height = 0.f;
        this->depth = 0.f;
        this->cls = -1;
    }

	// Top left corner is considered as origin
    CyC_INT         id;
	CPose    		origin;
	float		width;
	float		height;
	float		depth;
	CyC_INT	        cls;

    // Key of the image source filter for which the ROI was computed
    CycDatablockKey key;
};
typedef std::vector<CycBBox3D> CycBBoxes3D;

typedef octomap::CcrOcTree CcrOcTree;
struct CycEnvironment
{
    CPose                       pose;
    CycBBoxes3D                 objects;
    std::unique_ptr<CcrOcTree>  pOccupancyModel;

    CycEnvironment(float _resolution)
    {
        pOccupancyModel = std::make_unique<CcrOcTree>(_resolution);
    }

    CycEnvironment(const CycEnvironment& from)
    {
        from.copyTo(*this);
    }

    CycEnvironment(CycEnvironment&&) = default;

    CycEnvironment& operator=(const CycEnvironment& from)
    {
        if (this != &from)
        {
            from.copyTo(*this);
        }

        return *this;
    }

    CycEnvironment& operator=(CycEnvironment&&) = default;

    void copyTo(CycEnvironment& dest) const
    {
        dest.pose = pose;
        dest.objects = objects;
        dest.pOccupancyModel = std::make_unique<CcrOcTree>(pOccupancyModel->getResolution());

        // Information about color is lost when the copy constructor/operator is used
        // Maybe implement an efficient copy constructor for ColorOcTree?
        for (auto it = pOccupancyModel->begin_leafs(); it != pOccupancyModel->end_leafs(); ++it)
        {
            auto* node = dest.pOccupancyModel->updateNode(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z(), true, true);
            node->setValue(it->getValue());
            node->setColor(it->getColor());
            node->setObjectClass(it->getObjectClass());
        }
    }
};

// TODO: use CycReferenceSetpoints instead
using CycTrajectory = std::vector<Eigen::VectorXf>;

struct CycLane
{
    CycLane()
    {
        this->id = -1;
    }

    CycLane(CyC_INT _id)
    {
        this->id = _id;
    }

    CycLane(CyC_INT _id, Eigen::Vector4f _model)
    {
        this->id = _id;
        this->model = _model;
    }

    CyC_INT         id;
    Eigen::Vector4f model;
};
typedef std::vector<CycLane> CycLanesModel;

/*
 * SLAM types
 */
struct CycSlam
{
    CyC_TIME_UNIT           timestamp       = -1;
    CyC_LONG                id              = -1;
    CyC_LONG                ref_keyframe_id = -1;
    CyC_INT                 num_keyframes   = 0;
    CyC_INT                 num_map_points  = 0;
    bool                    is_keyframe     = false;
    bool                    is_mapping      = true;

    CPose                   Absolute_Cam_C;     // Camera pose in camera coordinates
    CPose                   Absolute_Imu_W;     // IMU pose in world coordinates
    CPose                   Absolute_Body_W;    // Body pose of the robot/vehicle in world coordinates
    Eigen::Vector3f         Bias_Acc_I = Eigen::Vector3f::Zero();
    Eigen::Vector3f         Bias_Gyro_I = Eigen::Vector3f::Zero();
    Eigen::Vector3f         Velocity_W = Eigen::Vector3f::Zero();

    std::vector<CyC_LONG>   local_frames;       // Frames that share map points with the current frame
    std::vector<CyC_LONG>   neighboring_frames; // Frames that share map points with local frames, but not with the current frame
    std::vector<CyC_INT>    latest_map_points;  // Latest map points added
    std::vector<CycImu>     imu_cache;          // Cached inertial data between the previous and the current frames

    std::vector<std::pair<CycVoxel, CycPoint>>  rel_map_points_W;   // Map points (and their observations) visible in the current
                                                                    // frame in world coordinates, relative to the camera pose
    std::vector<std::pair<CyC_LONG, CPose>>     prev_poses_Body_W;  // Pairs of frame IDs and poses
    std::vector<CyC_INT>                        prev_poses_type;    // 0: unrelated, 1: local frame, 2: neighboring frame
};

/*
 * Deep learning types
 */
struct CycDnnBranchIO
{
    CycDnnBranchIO()
    {
        this->id = CyC_UNDEFINED;
        this->data_type = CyC_UNDEFINED;
    }

    CyC_INT       id;
    CyC_DATA_TYPE data_type;

    std::vector<
        std::variant<
            std::vector<float>,
            std::vector<CyC_INT>,
            CycRois2D,
            CycBBoxes3D,
            CycLanesModel,
            CycImages,
            CycPoints
        >
    > data;
};
typedef std::vector<CycDnnBranchIO> CycDnnIO;

/*
 * Static vectors to be used as default function arguments
 */
static CycPoints                CycPoints_DEFAULT;
static std::vector<bool>        CyC_Bools_DEFAULT;
static std::vector<int>         CyC_INTS_DEFAULT;
static std::vector<float>       CyC_Floats_DEFAULT;
static std::unordered_set<int>  CyC_Unordered_Set_INT_DEFAULT;
static Eigen::Vector3f          CyC_Eigen_Vector3f_DEFAULT;

#endif /* CyC_TYPES_H_ */
