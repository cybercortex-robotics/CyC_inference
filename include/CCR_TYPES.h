// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CCR_TYPES_H_
#define CCR_TYPES_H_

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

// If compiler >= C++ 17, then use the std::variant implementation, otherwise use the mpark implementation stored in ccr include
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

typedef uint32_t                        CCR_UINT;
typedef int32_t                         CCR_INT;
typedef uint64_t                        CCR_ULONG;
typedef int64_t                         CCR_LONG;
typedef std::chrono::milliseconds::rep  CCR_TIME_UNIT;
typedef std::atomic<bool>               CCR_ATOMIC_BOOL;
typedef std::atomic<int>                CCR_ATOMIC_INT;
typedef std::atomic<double>             CCR_ATOMIC_DOUBLE;
typedef std::atomic<CCR_TIME_UNIT>      CCR_ATOMIC_TIME_UNIT;

// Data types
enum CCR_DATA_TYPE
{
    // Base types
    CCR_UNDEFINED			    = -1,
    CCR_RAW_DATA_STREAM	        = 0,
    CCR_VECTOR_BOOL             = 28,
    CCR_VECTOR_INT              = 1,
    CCR_VECTOR_FLOAT            = 2,
    CCR_VECTOR_DOUBLE           = 3,
    CCR_VECTOR_STRING           = 25,

    // Sensor data types
    CCR_IMAGE                   = 4,
    CCR_RADAR                   = 5,
    CCR_ULTRASONICS             = 6,
    CCR_IMU                     = 7,
    CCR_GPS                     = 8,

    // Perception types
    CCR_POINTS                  = 9,
    CCR_VOXELS                  = 10,
    CCR_POSES_6D                = 11,
    CCR_2D_ROIS                 = 12,
    CCR_3D_BBOXES               = 13,
    CCR_POINT_FEATURES          = 14,
    CCR_OCTREE                  = 15,
    CCR_GRIDMAP                 = 16,
    CCR_LANES_MODEL             = 17,
    CCR_SLAM                    = 23,

    // Deep learning types
    CCR_DNN_OUTPUT              = 18,

    // Control types
    CCR_REFERENCE_SETPOINTS     = 19,
    CCR_STATE                   = 20,
    CCR_CONTROL_INPUT           = 21,
    CCR_MEASUREMENT             = 22,
    CCR_ESTIMATED_TRAJECTORY    = 24,
    CCR_LANDMARKS               = 26,

    // Communication types
    CCR_TERMINAL_DATA           = 27,
};

// Filter types
using CCR_FILTER_TYPE = size_t;
static std::hash<std::string> CCR_FILTER_HashFunc;

/*
 * Datablock key used for storing entries in the Datablock
 * An entry is defined by the VisionCore's ID and the Filter's ID
 * The Datablock key must be unique
 */
struct CcrDatablockKey
{
    CCR_ULONG   nCoreID;
    CCR_INT     nFilterID;
    std::string sDescription;

    CcrDatablockKey() :
        nCoreID(0),
        nFilterID(-1)
    {}

    CcrDatablockKey(CCR_INT _core_id, CCR_INT _filter_id, std::string _description = "") :
        nCoreID(_core_id),
        nFilterID(_filter_id),
        sDescription(_description)
    {}

    bool operator==(const CcrDatablockKey& key) const
    {
        return (nCoreID == key.nCoreID && nFilterID == key.nFilterID);
    }
};
typedef std::vector<CcrDatablockKey> CcrDatablockKeys;

// Hash key for undordered maps based on CcrDatablockKey
template <>
struct std::hash<CcrDatablockKey>
{
    std::size_t operator()(const CcrDatablockKey& k) const
    {
        using std::size_t;
        using std::hash;

        // Compute individual hash values for nCoreID and nFilterID and combine them using XOR and bit shifting
        return ((std::hash<CCR_ULONG>()(k.nCoreID) ^ (std::hash<CCR_INT>()(k.nFilterID) << 1)) >> 1);
    }
};

/*
 * Control types
 */
struct CcrReferenceSetPoints
{
    // Reference vector calculated via planning (control reference)
    std::vector<Eigen::VectorXf> ref;

    // Samples of reference state trajectories (used mainly for visualization)
    std::vector<std::vector<Eigen::VectorXf>> ref_samples;

    // unique ID for the reference trajectory
    CCR_INT id;
};

struct CcrLandmark
{
    CPose                           pose;
    float                           travel_time = 0.f;
    std::vector<Eigen::Vector4f>    waypoints;
};
typedef std::unordered_map<CCR_INT, CcrLandmark> CcrLandmarks;

struct CcrTerminalCommand
{
    std::vector<CCR_INT>    cmd;
    CcrLandmarks            landmarks;
    CPose                   destination;
};

struct CcrState
{
    CcrState()
    {}

    CcrState(const CCR_UINT _num_state_variables)
    {
        this->x_hat = Eigen::VectorXf::Zero(_num_state_variables);
    }

    friend auto operator<<(std::ostream& _os, CcrState const& _state) -> std::ostream&
    {
        std::string str;
        for (CCR_INT i = 0; i < _state.x_hat.size() - 1; ++i)
            str.append(std::to_string(_state.x_hat[i]) + "\t");

        if (_state.x_hat.size() > 0)
            str.append(std::to_string(_state.x_hat[_state.x_hat.size() - 1]));

        return _os << "State: [" << str << "]";
    }

    Eigen::VectorXf x_hat; // State vector
};

struct CcrMeasurement
{
    CcrMeasurement()
    {}

    CcrMeasurement(const CCR_UINT _num_measurement_variables)
    {
        this->y_hat = Eigen::VectorXf::Zero(_num_measurement_variables);
    }

    Eigen::VectorXf y_hat; // Measurement vector (output)
};

struct CcrControlInput
{
    CcrControlInput()
    {}

    CcrControlInput(const CCR_UINT _num_control_inputs)
    {
        this->u = Eigen::VectorXf::Zero(_num_control_inputs);
    }
    // Control input calculated for setpoint 
    Eigen::VectorXf                 u; // u = [thrust, roll_torque, pitch_torque, yaw_torque]
    CcrReferenceSetPoints           ref_pts;
    std::vector<Eigen::VectorXf>    goal_points;
};

/*
 * Sensors data types
 */
 // --- 6D poses vector ---
typedef std::vector<CPose> CcrPoses;

// --- IMU ---
struct CcrImu
{
    CcrImu()
    {
        acc = Eigen::Vector3f{ 0.f, 0.f, 0.f };
        gyro = Eigen::Vector3f{ 0.f, 0.f, 0.f };
        magnet = Eigen::Vector3f{ 0.f, 0.f, 0.f };
        this->timestamp = -1;
    }

    CcrImu(const Eigen::Vector3f& _acc, const Eigen::Vector3f& _gyro, const CCR_TIME_UNIT& _timestamp = -1) :
        acc(_acc), gyro(_gyro), timestamp(_timestamp)
    {
        magnet = Eigen::Vector3f{ 0.f, 0.f, 0.f };
    }

    CcrImu(const Eigen::Vector3f& _acc, const Eigen::Vector3f& _gyro, const Eigen::Vector3f& _magnet, const CCR_TIME_UNIT& _timestamp = -1) :
        acc(_acc), gyro(_gyro), magnet(_magnet), timestamp(_timestamp)
    {}

    friend auto operator<<(std::ostream& _os, CcrImu const& _imu) -> std::ostream&
    {
        return _os << _imu.timestamp << ":\t" << _imu.acc.x() << "\t" << _imu.acc.y() << "\t" << _imu.acc.z() << "\t(" << _imu.gyro.x() << "\t" << _imu.gyro.y() << "\t" << _imu.gyro.z() << ")";
    }

    CCR_TIME_UNIT   timestamp;
    Eigen::Vector3f acc;    // [m/s^2]
    Eigen::Vector3f gyro;   // [deg/s]
    Eigen::Vector3f magnet; // [uT]
    CQuaternion     quat;
};
typedef std::vector<CcrImu> CcrImus;

// --- Image ---
struct CcrImage_
{
    CCR_TIME_UNIT   nTimestamp;     // Image acquisition timestamp
    CCR_UINT	    nRows;		    // Number of image rows
    CCR_UINT	    nCols;		    // Number of image columns
    CCR_UINT	    nChannels1;     // Number of image channels for the first image
    CCR_UINT	    nChannels2;     // Number of image channels for the second image
    CCR_UINT	    nType1;		    // Image type, according to the OpenCV types
	CCR_UINT	    nType2;		    // Image type, for second image, according to the OpenCV types
    CCR_UINT	    nTotalSize1;    // Total image size
	CCR_UINT	    nTotalSize2;    // Total image size for second image
    bool	        bIsStereo;	    // True for stereo or RGB-D images
    void*		    pData1;		    // Pointer to the RGB image data, or left RGB image in case of a stereo image
    void*		    pData2;		    // Pointer to the right RGB image in case of a stereo image, or depth image in case of RGB-D images
    CcrDatablockKey key;            // Key of the image source filter

	CcrImage_() : 
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

    explicit CcrImage_(const cv::Mat& _rgb, const CCR_TIME_UNIT& _ts = -1) :
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
        nTotalSize1 = (CCR_UINT)(_rgb.total() * _rgb.elemSize());

        pData1 = malloc(nTotalSize1);
        memcpy(pData1, _rgb.data, nTotalSize1);
    }

    explicit CcrImage_(const cv::Mat& _rgb, const cv::Mat& _depth, const CCR_TIME_UNIT& _ts = -1) :
        CcrImage_(_rgb, _ts)
    {
        bIsStereo = true;

        nChannels2 = _depth.channels();
        nType2 = _depth.type();
        nTotalSize2 = (CCR_UINT)(_depth.total() * _depth.elemSize());

        pData2 = malloc(nTotalSize2);
        memcpy(pData2, _depth.data, nTotalSize2);
    }

	CcrImage_(const CcrImage_& _other):
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

	CcrImage_(CcrImage_&& _other) noexcept:
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

	CcrImage_& operator=(CcrImage_&& _other) noexcept
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

	CcrImage_& operator=(const CcrImage_& _other)
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

    ~CcrImage_()
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
typedef std::vector<CcrImage_> CcrImages;

// --- 2D image point ---
struct CcrPoint
{
    CcrPoint() :
        id(-1), score(-1.f), depth(-1.f), angle(0.f)
    {
        pt2d = Eigen::Vector2f{ -1.f, -1.f };
        key = CcrDatablockKey{ -1, -1 };
    }

    CcrPoint(Eigen::Vector2f _pt2D, float _depth = -1.f, CCR_INT _id = -1, float _score = -1.f, cv::Mat _descriptor = cv::Mat(), float _angle = 0.f, CcrDatablockKey _key = CcrDatablockKey{ -1, -1 }) :
        pt2d(_pt2D), depth(_depth), id(_id), score(_score), key(_key), descriptor(_descriptor), angle(_angle)
    {}
	
	CcrPoint(float _x, float _y, float _depth = -1.f, CCR_INT _id = -1, float _score = -1.f, cv::Mat _descriptor = cv::Mat(), float _angle = 0.f, CcrDatablockKey _key = CcrDatablockKey{ -1, -1 }) :
        depth(_depth), id(_id), score(_score), key(_key), descriptor(_descriptor), angle(_angle)
    {
		pt2d = Eigen::Vector2f {_x, _y };
	}

    friend auto operator<<(std::ostream& _os, CcrPoint const& _pt) -> std::ostream&
    {
        return _os << _pt.id << ":\t" << _pt.pt2d.x() << "\t" << _pt.pt2d.y() << "\t" << _pt.depth << "\t(score: " << _pt.score << ", angle: " << _pt.angle << ")";
    }
	
	Eigen::Vector2f pt2d;
    float           depth;
	CCR_INT		    id;
    float           score;

    cv::Mat         descriptor;
    float           angle;

    CcrDatablockKey key;  // Key of the image source filter
};
typedef std::vector<CcrPoint> CcrPoints;

// --- Voxels (point cloud) ---
struct CcrVoxel
{
    CcrVoxel() :
        id(-1)
    {
        pt3d = Eigen::Vector4f{ 0.f, 0.f, 0.f, 0.f };
    }

    CcrVoxel(Eigen::Vector4f _pt3D, CCR_INT _id = -1, float _score = -1.f, float _angle = 0.f, float _error = 9999.f) :
        pt3d(_pt3D), id(_id), error(_error)
    {}

    CcrVoxel(float _x, float _y, float _z, float _w = 1.f, CCR_INT _id = -1, float _error = 9999.f) :
        id(_id), error(_error)
    {
        pt3d = Eigen::Vector4f{ _x, _y, _z, _w };
    }

    friend auto operator<<(std::ostream& _os, CcrVoxel const& _vx) -> std::ostream&
    {
        return _os << _vx.id << ":\t" << _vx.pt3d.x() << "\t" << _vx.pt3d.y() << "\t" << _vx.pt3d.z() << "\t(err: " << _vx.error << ")";
    }

    Eigen::Vector4f pt3d;
    CCR_INT		    id;
    float           error;
};
typedef std::vector<CcrVoxel> CcrVoxels;

// --- Ultrasonics ---
struct CcrUltrasonic
{
    float   range;
    CPose   pose;
	float   max_range;
};
typedef std::vector<CcrUltrasonic> CcrUltrasonics;

// --- GPS ---
struct CcrGps
{
    float baseLat;
    float baseLng;
    float baseAlt;
    float lat;
    float lng;
    float alt;
};

// TBD: use CcrVoxel instead
struct CcrLidarPoint
{
    float angle;
    float distance;
    float quality;
};
typedef std::vector<CcrLidarPoint> CcrLidarPoints;

/*
 * Perception types
 */
struct CcrRoi2D
{
    CcrRoi2D(Eigen::Vector2f _origin, float _width, float _height, float _conf = 0.F, CCR_INT _cls = -1, CCR_INT _id = -1)
    {
        this->cls = _cls; 
        this->id = _id;
        this->confidence = _conf;
        this->origin = std::move(_origin);
        this->width = _width;
        this->height = _height;
    }

    CcrRoi2D()
    {
        this->cls = -1; 
        this->id = -1;
        this->confidence = 0.F;
        this->origin = Eigen::Vector2f(0, 0);
        this->width = 0;
        this->height = 0;
    }

    friend auto operator<<(std::ostream& _os, CcrRoi2D const& _roi) -> std::ostream&
    {
        return _os << _roi.id << "\t[" << _roi.key.nCoreID << ", " << _roi.key.nFilterID << "]:\t" << _roi.origin.x() << "\t" << _roi.origin.y() << "\t" << _roi.width << "\t" << _roi.height << "\t(" << _roi.cls << ", " << _roi.confidence << ")";
    }

    // Top left corner is considered as origin
    CCR_INT         id;
    CCR_INT         cls;
    float       confidence;
    Eigen::Vector2f origin;
    float       width;
    float       height;

    // Key of the image source filter for which the ROI was computed
    CcrDatablockKey key;
};
typedef std::vector<CcrRoi2D> CcrRois2D;

struct CcrBBox3D
{
	CcrBBox3D(CPose _origin, float _width, float _height, float _depth,  CCR_INT _cls = -1, CCR_INT _id = -1)
	{
        this->id = _id;
		this->origin = std::move(_origin);
		this->width = _width;
		this->height = _height;
		this->depth = _depth;
		this->cls = _cls;
	}

	CcrBBox3D()
    {
        this->id = -1;
        this->width = 0.f;
        this->height = 0.f;
        this->depth = 0.f;
        this->cls = -1;
    }

	// Top left corner is considered as origin
    CCR_INT         id;
	CPose    		origin;
	float		width;
	float		height;
	float		depth;
	CCR_INT	        cls;

    // Key of the image source filter for which the ROI was computed
    CcrDatablockKey key;
};
typedef std::vector<CcrBBox3D> CcrBBoxes3D;

typedef octomap::CcrOcTree CcrOcTree;
struct CcrEnvironment
{
    CPose                       pose;
    CcrBBoxes3D                 objects;
    std::unique_ptr<CcrOcTree>  pOccupancyModel;

    CcrEnvironment(float _resolution)
    {
        pOccupancyModel = std::make_unique<CcrOcTree>(_resolution);
    }

    CcrEnvironment(const CcrEnvironment& from)
    {
        from.copyTo(*this);
    }

    CcrEnvironment(CcrEnvironment&&) = default;

    CcrEnvironment& operator=(const CcrEnvironment& from)
    {
        if (this != &from)
        {
            from.copyTo(*this);
        }

        return *this;
    }

    CcrEnvironment& operator=(CcrEnvironment&&) = default;

    void copyTo(CcrEnvironment& dest) const
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

// TODO: use CcrReferenceSetpoints instead
using CcrTrajectory = std::vector<Eigen::VectorXf>;

struct CcrLane
{
    CcrLane()
    {
        this->id = -1;
    }

    CcrLane(CCR_INT _id)
    {
        this->id = _id;
    }

    CcrLane(CCR_INT _id, Eigen::Vector4f _model)
    {
        this->id = _id;
        this->model = _model;
    }

    CCR_INT         id;
    Eigen::Vector4f model;
};
typedef std::vector<CcrLane> CcrLanesModel;

/*
 * SLAM types
 */
struct CcrSlam
{
    CCR_TIME_UNIT           timestamp       = -1;
    CCR_LONG                id              = -1;
    CCR_LONG                ref_keyframe_id = -1;
    bool                    is_keyframe     = false;
    CCR_INT                 num_keyframes   = 0;
    CCR_INT                 num_map_points  = 0;

    CPose                   Absolute_Cam_C;     // Camera pose in camera coordinates
    CPose                   Absolute_Imu_I;     // IMU pose in IMU coordinates
    CPose                   Absolute_Body_W;    // Absolute pose of the robot/vehicle in world coordinates

    std::vector<CCR_LONG>   local_frames;       // Frames that share map points with the current frame
    std::vector<CCR_LONG>   neighboring_frames; // Frames that share map points with local frames, but not with the current frame
    std::vector<CCR_INT>    latest_map_points;  // Latest map points added
    std::vector<CcrImu>     imu_cache;          // Cached inertial data between the previous and the current frames

    std::vector<std::pair<CcrVoxel, CcrPoint>>  rel_map_points_W;   // Map points (and their observations) visible in the current
                                                                    // frame in world coordinates, relative to the camera pose
    std::vector<std::pair<CCR_LONG, CPose>>     prev_poses_Body_W;  // Pairs of frame IDs and poses
    std::vector<CCR_INT>                        prev_poses_type;    // 0: unrelated, 1: local frame, 2: neighboring frame
};

/*
 * Deep learning types
 */
struct CcrDnnBranchIO
{
    CcrDnnBranchIO()
    {
        this->id = CCR_UNDEFINED;
        this->data_type = CCR_UNDEFINED;
    }

    CCR_INT       id;
    CCR_DATA_TYPE data_type;

    std::vector<
        std::variant<
            std::vector<float>,
            std::vector<CCR_INT>,
            CcrRois2D,
            CcrBBoxes3D,
            CcrLanesModel,
            CcrImages,
            CcrPoints
        >
    > data;
};
typedef std::vector<CcrDnnBranchIO> CcrDnnIO;

/*
 * Static vectors to be used as default function arguments
 */
static CcrPoints                CcrPoints_DEFAULT;
static std::vector<bool>        boolS_DEFAULT;
static std::vector<int>         CCR_INTS_DEFAULT;
static std::vector<float>       floatS_DEFAULT;
static std::unordered_set<int>  CCR_Unordered_Set_INT_DEFAULT;
static Eigen::Vector3f          CCR_Eigen_Vector3f_DEFAULT;

#endif /* CCR_TYPES_H_ */
