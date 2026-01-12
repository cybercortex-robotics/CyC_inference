// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CConversions.h"

CConversions::CConversions()
{}

CConversions::~CConversions()
{}

std::string CConversions::DataType2String(CyC_DATA_TYPE nDataType)
{
	std::string sDataTypeName;

	switch (nDataType)
	{
	case CyC_UNDEFINED:
		sDataTypeName = "Undef";
		break;

	case CyC_RAW_DATA_STREAM:
		sDataTypeName = "Raw";
		break;

	case CyC_VECTOR_BOOL:
		sDataTypeName = "Bools";
		break;

    case CyC_VECTOR_INT:
        sDataTypeName = "Ints";
        break;

    case CyC_VECTOR_FLOAT:
        sDataTypeName = "Floats";
        break;

    case CyC_VECTOR_DOUBLE:
        sDataTypeName = "Doubles";
        break;

	case CyC_VECTOR_STRING:
		sDataTypeName = "Strings";
		break;

	case CyC_POINTS:
		sDataTypeName = "Pts 2D";
		break;

	case CyC_VOXELS:
		sDataTypeName = "Voxels";
		break;

    case CyC_POSES_6D:
		sDataTypeName = "Pose6D";
		break;

	case CyC_IMAGE:
		sDataTypeName = "Image";
		break;

	case CyC_2D_ROIS:
        sDataTypeName = "2D ROIs";
        break;

    case CyC_3D_BBOXES:
        sDataTypeName = "3D BBs";
        break;

    case CyC_LANES_MODEL:
        sDataTypeName = "Lanes";
        break;

	case CyC_GPS:
		sDataTypeName = "GPS";
		break;

	case CyC_IMU:
		sDataTypeName = "IMU";
		break;

    case CyC_ULTRASONICS:
		sDataTypeName = "USonic";
		break;

    case CyC_OCTREE:
        sDataTypeName = "OcTree";
        break;

    /*
     * Deep learning types
     */
    case CyC_DNN_OUTPUT:
        sDataTypeName = "DNN";
        break;

    /*
     * Control types
     */
    case CyC_REFERENCE_SETPOINTS:
        sDataTypeName = "Ref.";
        break;

    case CyC_STATE:
        sDataTypeName = "State";
        break;

    case CyC_CONTROL_INPUT:
        sDataTypeName = "Contrl";
        break;

    case CyC_LANDMARKS:
		sDataTypeName = "Lndmrks";
		break;

	case CyC_TERMINAL_DATA:
		sDataTypeName = "Term";
		break;

	default:
		sDataTypeName = "Undef";
		break;
	}

	return sDataTypeName;
}

CyC_DATA_TYPE CConversions::String2DataType(const std::string& sDataType)
{
	CyC_DATA_TYPE nDataType(CyC_UNDEFINED);

	if (strcmp(sDataType.c_str(), "CyC_UNDEFINED") == 0)
	{
		nDataType = CyC_UNDEFINED;
	}
	else if (strcmp(sDataType.c_str(), "CyC_VECTOR_BOOL") == 0 || strcmp(sDataType.c_str(), "ROVIS_VECTOR_BOOL") == 0)
	{
		nDataType = CyC_VECTOR_BOOL;
	}
	else if (strcmp(sDataType.c_str(), "CyC_VECTOR_INT") == 0 || strcmp(sDataType.c_str(), "ROVIS_VECTOR_INT") == 0)
	{
		nDataType = CyC_VECTOR_INT;
	}
	else if (strcmp(sDataType.c_str(), "CyC_VECTOR_FLOAT") == 0 || strcmp(sDataType.c_str(), "ROVIS_VECTOR_FLOAT") == 0)
	{
		nDataType = CyC_VECTOR_FLOAT;
	}
	else if (strcmp(sDataType.c_str(), "CyC_VECTOR_DOUBLE") == 0 || strcmp(sDataType.c_str(), "ROVIS_VECTOR_DOUBLE") == 0)
	{
		nDataType = CyC_VECTOR_DOUBLE;
	}
	else if (strcmp(sDataType.c_str(), "CyC_VECTOR_STRING") == 0 || strcmp(sDataType.c_str(), "ROVIS_VECTOR_STRING") == 0)
	{
		nDataType = CyC_VECTOR_STRING;
	}
	else if (strcmp(sDataType.c_str(), "CyC_IMAGE") == 0 || strcmp(sDataType.c_str(), "ROVIS_IMAGE") == 0)
	{
		nDataType = CyC_IMAGE;
	}
	else if (strcmp(sDataType.c_str(), "CyC_RADAR") == 0)
	{
		nDataType = CyC_RADAR;
	}
	else if (strcmp(sDataType.c_str(), "CyC_ULTRASONICS") == 0)
	{
		nDataType = CyC_ULTRASONICS;
	}
	else if (strcmp(sDataType.c_str(), "CyC_IMU") == 0)
	{
		nDataType = CyC_IMU;
	}
	else if (strcmp(sDataType.c_str(), "CyC_GPS") == 0)
	{
		nDataType = CyC_GPS;
	}
	else if (strcmp(sDataType.c_str(), "CyC_POINTS") == 0 || strcmp(sDataType.c_str(), "ROVIS_POINTS") == 0)
	{
		nDataType = CyC_POINTS;
	}
	else if (strcmp(sDataType.c_str(), "CyC_VOXELS") == 0)
	{
		nDataType = CyC_VOXELS;
	}
	else if (strcmp(sDataType.c_str(), "CyC_POSES_6D") == 0)
	{
		nDataType = CyC_POSES_6D;
	}
	else if (strcmp(sDataType.c_str(), "CyC_2D_ROIS") == 0 || strcmp(sDataType.c_str(), "ROVIS_2D_ROIS") == 0)
	{
		nDataType = CyC_2D_ROIS;
	}
	else if (strcmp(sDataType.c_str(), "CyC_3D_BBOXES") == 0)
	{
		nDataType = CyC_3D_BBOXES;
	}
	else if (strcmp(sDataType.c_str(), "CyC_POINT_FEATURES") == 0)
	{
		nDataType = CyC_POINT_FEATURES;
	}
	else if (strcmp(sDataType.c_str(), "CyC_OCTREE") == 0)
	{
		nDataType = CyC_OCTREE;
	}
	else if (strcmp(sDataType.c_str(), "CyC_GRIDMAP") == 0)
	{
		nDataType = CyC_GRIDMAP;
	}
	else if (strcmp(sDataType.c_str(), "CyC_LANES_MODEL") == 0 || strcmp(sDataType.c_str(), "ROVIS_LANES_MODEL") == 0)
	{
		nDataType = CyC_LANES_MODEL;
	}
	else if (strcmp(sDataType.c_str(), "CyC_SLAM") == 0)
	{
		nDataType = CyC_SLAM;
	}
	else if (strcmp(sDataType.c_str(), "CyC_DNN_OUTPUT") == 0)
	{
		nDataType = CyC_DNN_OUTPUT;
	}
	else if (strcmp(sDataType.c_str(), "CyC_REFERENCE_SETPOINTS") == 0)
	{
		nDataType = CyC_REFERENCE_SETPOINTS;
	}
	else if (strcmp(sDataType.c_str(), "CyC_STATE") == 0)
	{
		nDataType = CyC_STATE;
	}
	else if (strcmp(sDataType.c_str(), "CyC_CONTROL_INPUT") == 0)
	{
		nDataType = CyC_CONTROL_INPUT;
	}
	else if (strcmp(sDataType.c_str(), "CyC_MEASUREMENT") == 0)
	{
		nDataType = CyC_MEASUREMENT;
	}
	else if (strcmp(sDataType.c_str(), "CyC_LANDMARKS") == 0)
	{
		nDataType = CyC_LANDMARKS;
	}
	else if (strcmp(sDataType.c_str(), "CyC_TERMINAL_DATA") == 0)
	{
		nDataType = CyC_TERMINAL_DATA;
	}
	else if (strcmp(sDataType.c_str(), "CyC_ESTIMATED_TRAJECTORY") == 0)
	{
		nDataType = CyC_ESTIMATED_TRAJECTORY;
	}

	return nDataType;
}
