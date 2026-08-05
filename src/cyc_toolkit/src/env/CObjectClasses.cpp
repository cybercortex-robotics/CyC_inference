// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CObjectClasses.h"

const cv::Scalar color::colormap[155] = { // BGR!
    /*   0 */   {170, 100, 60},
    /*   1 */   {55, 170, 55},
    /*   2 */   {31, 170, 250},
    /*   3 */   {255, 0, 0},
    /*   4 */   {15, 15, 255},
    /*   5 */   {65, 90, 210},
    /*   6 */   {255, 75, 75},
    /*   7 */   {125, 0, 0},
    /*   8 */   {255, 255, 255},
    /*   9 */   {0, 255, 255},
    /*  10 */   {250, 170, 34},
    /*  11 */   {0, 0, 200},
    /*  12 */   {0, 150, 20},
    /*  13 */   {102, 102, 156},
    /*  14 */   {128, 64, 255},
    /*  15 */   {140, 140, 200},
    /*  16 */   {170, 170, 170},
    /*  17 */   {250, 170, 36},
    /*  18 */   {250, 170, 160},
    /*  19 */   {55, 250, 37},
    /*  20 */   {96, 96, 96},
    /*  21 */   {230, 150, 140},
    /*  22 */   {128, 64, 128},
    /*  23 */   {50, 110, 200},
    /*  24 */   {110, 110, 110},
    /*  25 */   {244, 35, 232},
    /*  26 */   {128, 196, 128},
    /*  27 */   {150, 100, 100},
    /*  28 */   {70, 70, 70},
    /*  29 */   {150, 150, 150},
    /*  30 */   {150, 120, 90},
    /*  31 */   {220, 20, 60},
    /*  32 */   {220, 20, 60},
    /*  33 */   {255, 0, 0},
    /*  34 */   {255, 0, 100},
    /*  35 */   {255, 0, 200},
    /*  36 */   {255, 255, 255},
    /*  37 */   {255, 255, 255},
    /*  38 */   {250, 170, 29},
    /*  39 */   {250, 170, 28},
    /*  40 */   {250, 170, 26},
    /*  41 */   {250, 170, 25},
    /*  42 */   {250, 170, 24},
    /*  43 */   {250, 170, 22},
    /*  44 */   {250, 170, 21},
    /*  45 */   {250, 170, 20},
    /*  46 */   {255, 255, 255},
    /*  47 */   {250, 170, 19},
    /*  48 */   {250, 170, 18},
    /*  49 */   {250, 170, 12},
    /*  50 */   {250, 170, 11},
    /*  51 */   {255, 255, 255},
    /*  52 */   {255, 255, 255},
    /*  53 */   {250, 170, 16},
    /*  54 */   {250, 170, 15},
    /*  55 */   {250, 170, 15},
    /*  56 */   {255, 255, 255},
    /*  57 */   {255, 255, 255},
    /*  58 */   {0, 255, 0},
    /*  59 */   {255, 255, 255},
    /*  60 */   {64, 170, 64},
    /*  61 */   {230, 160, 50},
    /*  62 */   {70, 130, 180},
    /*  63 */   {190, 255, 255},
    /*  64 */   {152, 251, 152},
    /*  65 */   {107, 142, 35},
    /*  66 */   {0, 170, 30},
    /*  67 */   {255, 255, 128},
    /*  68 */   {250, 0, 30},
    /*  69 */   {100, 140, 180},
    /*  70 */   {220, 128, 128},
    /*  71 */   {222, 40, 40},
    /*  72 */   {100, 170, 30},
    /*  73 */   {40, 40, 40},
    /*  74 */   {33, 33, 33},
    /*  75 */   {100, 128, 160},
    /*  76 */   {20, 20, 255},
    /*  77 */   {142, 0, 0},
    /*  78 */   {70, 100, 150},
    /*  79 */   {250, 171, 30},
    /*  80 */   {250, 172, 30},
    /*  81 */   {250, 173, 30},
    /*  82 */   {250, 174, 30},
    /*  83 */   {250, 175, 30},
    /*  84 */   {250, 176, 30},
    /*  85 */   {210, 170, 100},
    /*  86 */   {153, 153, 153},
    /*  87 */   {153, 153, 153},
    /*  88 */   {128, 128, 128},
    /*  89 */   {0, 0, 80},
    /*  90 */   {210, 60, 60},
    /*  91 */   {250, 170, 30},
    /*  92 */   {250, 170, 30},
    /*  93 */   {250, 170, 30},
    /*  94 */   {250, 170, 30},
    /*  95 */   {250, 170, 30},
    /*  96 */   {250, 170, 30},
    /*  97 */   {192, 192, 192},
    /*  98 */   {192, 192, 192},
    /*  99 */   {192, 192, 192},
    /* 100 */   {220, 220, 0},
    /* 101 */   {220, 220, 0},
    /* 102 */   {0, 0, 196},
    /* 103 */   {192, 192, 192},
    /* 104 */   {220, 220, 0},
    /* 105 */   {140, 140, 20},
    /* 106 */   {119, 11, 32},
    /* 107 */   {150, 0, 255},
    /* 108 */   {0, 60, 100},
    /* 109 */   {0, 0, 142},
    /* 110 */   {0, 0, 90},
    /* 111 */   {0, 0, 230},
    /* 112 */   {0, 80, 100},
    /* 113 */   {128, 64, 64},
    /* 114 */   {0, 0, 110},
    /* 115 */   {0, 0, 70},
    /* 116 */   {0, 0, 142},
    /* 117 */   {0, 0, 192},
    /* 118 */   {170, 170, 170},
    /* 119 */   {32, 32, 32},
    /* 120 */   {111, 74, 0},
    /* 121 */   {120, 10, 10},
    /* 122 */   {81, 0, 81},
    /* 123 */   {111, 111, 0},
    /* 124 */   {0, 0, 0},
    /* 125 */   {150, 150, 150},
    /* 126 */   {150, 120, 90},
    /* 127 */   {220, 20, 60},
    /* 128 */   {220, 20, 60},
    /* 129 */   {255, 0, 0},
    /* 130 */   {255, 0, 100},
    /* 131 */   {255, 0, 200},
    /* 132 */   {255, 255, 255},
    /* 133 */   {255, 255, 255},
    /* 134 */   {250, 170, 29},
    /* 135 */   {250, 170, 28},
    /* 136 */   {250, 170, 26},
    /* 137 */   {250, 170, 25},
    /* 138 */   {250, 170, 24},
    /* 139 */   {250, 170, 22},
    /* 140 */   {250, 170, 21},
    /* 141 */   {250, 170, 20},
    /* 142 */   {255, 255, 255},
    /* 143 */   {250, 170, 19},
    /* 144 */   {250, 170, 18},
    /* 145 */   {250, 170, 12},
    /* 146 */   {250, 170, 11},
    /* 147 */   {255, 255, 255},
    /* 148 */   {255, 255, 255},
    /* 149 */   {250, 170, 16},
    /* 150 */   {250, 170, 15},
    /* 151 */   {250, 170, 15},
    /* 152 */   {255, 255, 255},
    /* 153 */   {255, 255, 255},
    /* 154 */   {255, 255, 255}
};

cv::Scalar color::black     = cv::Scalar{ 0, 0, 0, 0 };
cv::Scalar color::gray      = cv::Scalar{ 120, 120, 120, 0 };
cv::Scalar color::white     = cv::Scalar{ 255, 255, 255, 0 };
cv::Scalar color::red       = cv::Scalar{ 50, 50, 255, 0 };
cv::Scalar color::green     = cv::Scalar{ 60, 250, 50, 0 };
cv::Scalar color::blue      = cv::Scalar{ 255, 243, 153, 0 };
cv::Scalar color::yellow    = cv::Scalar{ 50, 255, 255, 0 };
cv::Scalar color::pink      = cv::Scalar{ 255, 200, 255, 0 };
cv::Scalar color::cyc_background = cv::Scalar{ 35, 35, 35, 0 };

cv::Scalar color::x_axis = color::green;
cv::Scalar color::y_axis = color::blue;
cv::Scalar color::z_axis = color::red;

cv::Scalar color::undefined                = cv::Scalar{ 50, 50, 50, 0 };
cv::Scalar color::vehicle                  = cv::Scalar{ 255, 255, 255, 0 };
cv::Scalar color::mission_path             = cv::Scalar{ 0, 0, 192, 0 };
cv::Scalar color::reference_path           = cv::Scalar{ 119, 241, 84, 0 };
cv::Scalar color::current_path             = cv::Scalar{ 255, 100, 51, 0 };
cv::Scalar color::trajectories_samples     = cv::Scalar{ 188, 156, 3, 0 };
cv::Scalar color::free_space               = cv::Scalar{ 230, 230, 230, 0 };
cv::Scalar color::obstacle                 = cv::Scalar{ 191, 34, 0, 0 };
cv::Scalar color::obstacle_prediction      = cv::Scalar{ 220, 0, 220, 0 };
cv::Scalar color::road_model               = cv::Scalar{ 0, 255, 0, 0 };
cv::Scalar color::lidar                    = cv::Scalar{ 150, 153, 216, 0 };
cv::Scalar color::ultrasonics              = cv::Scalar{ 0, 0, 216, 0 };
cv::Scalar color::depth                    = cv::Scalar{ 234, 217, 153, 0 };
cv::Scalar color::slam_map_points          = cv::Scalar{ 0, 153, 0, 0 };
cv::Scalar color::wheel_stuck              = cv::Scalar{ 0, 0, 102, 0 };

cv::Scalar color::info                     = cv::Scalar{ 220, 200, 100, 0 };
cv::Scalar color::solutions                = cv::Scalar{ 20, 200, 230, 0 };
cv::Scalar color::best_solution            = cv::Scalar{ 20, 230, 30, 0 };
cv::Scalar color::curr_pts_2d              = cv::Scalar{ 255, 255, 255, 0 };
cv::Scalar color::prev_pts_2d              = cv::Scalar{ 28, 100, 200, 0 };
cv::Scalar color::reprojection             = cv::Scalar{ 0, 0, 255, 0 };
cv::Scalar color::orthogonal_projection    = cv::Scalar{ 60, 250, 50, 0 };
cv::Scalar color::reproj_pts_2d_pos_depth  = cv::Scalar{ 255, 70, 30, 0 };
cv::Scalar color::reproj_pts_2d_neg_depth  = cv::Scalar{ 55, 30, 240, 0 };
cv::Scalar color::epi_lines                = cv::Scalar{ 0, 255, 255, 0 };

bool color::fromName(const std::string& _name, cv::Scalar& _out)
{
    // Built on first use rather than at static initialization time: the members it reads are
    // statics of this same translation unit, and a namespace scope map would be racing them.
    static const std::unordered_map<std::string, const cv::Scalar*> palette = {
        { "black",                      &color::black },
        { "gray",                       &color::gray },
        { "grey",                       &color::gray },
        { "white",                      &color::white },
        { "red",                        &color::red },
        { "green",                      &color::green },
        { "blue",                       &color::blue },
        { "yellow",                     &color::yellow },
        { "pink",                       &color::pink },
        { "cyc_background",             &color::cyc_background },
        { "x_axis",                     &color::x_axis },
        { "y_axis",                     &color::y_axis },
        { "z_axis",                     &color::z_axis },
        { "undefined",                  &color::undefined },
        { "vehicle",                    &color::vehicle },
        { "mission_path",               &color::mission_path },
        { "reference_path",             &color::reference_path },
        { "current_path",               &color::current_path },
        { "trajectories_samples",       &color::trajectories_samples },
        { "free_space",                 &color::free_space },
        { "obstacle",                   &color::obstacle },
        { "obstacle_prediction",        &color::obstacle_prediction },
        { "road_model",                 &color::road_model },
        { "lidar",                      &color::lidar },
        { "ultrasonics",                &color::ultrasonics },
        { "depth",                      &color::depth },
        { "slam_map_points",            &color::slam_map_points },
        { "wheel_stuck",                &color::wheel_stuck },
        { "info",                       &color::info },
        { "solutions",                  &color::solutions },
        { "best_solution",              &color::best_solution },
        { "curr_pts_2d",                &color::curr_pts_2d },
        { "prev_pts_2d",                &color::prev_pts_2d },
        { "reprojection",               &color::reprojection },
        { "orthogonal_projection",      &color::orthogonal_projection },
        { "reproj_pts_2d_pos_depth",    &color::reproj_pts_2d_pos_depth },
        { "reproj_pts_2d_neg_depth",    &color::reproj_pts_2d_neg_depth },
        { "epi_lines",                  &color::epi_lines }
    };

    const auto it = palette.find(_name);
    if (it == palette.end())
    {
        return false;
    }

    _out = *it->second;
    return true;
}

CObjectClasses::CObjectClasses(const std::string& _object_classes_file)
{
    m_bIsInitialized = false;

	// Add static object classes
	addStaticClasses();

	// Check if file exists
	bool bFileExists = CFileUtils::FileExist(_object_classes_file.c_str());

	if (bFileExists)
	{
		libconfig::Config LibConfigFile;

		try
		{
			LibConfigFile.readFile(_object_classes_file.c_str());
			const libconfig::Setting& rootConfig = LibConfigFile.getRoot();
			const libconfig::Setting& ObjectClasses = rootConfig["ObjectClasses"];

			m_nNumClasses = ObjectClasses.getLength();

			for (CyC_UINT i = 0; i < m_nNumClasses; i++)
			{
				CyC_INT nClassID(-1);
				std::string sClassName = ObjectClasses[i].getName();

				ObjectClasses[i].lookupValue("ID", nClassID);
				m_ObjectClassesMap[nClassID] = sClassName;
			}
		}
		catch (libconfig::ParseException& ex)
		{
			spdlog::error("Failed to read configuration with error: {} at line {}", ex.getError(), ex.getLine());
		}

		m_bIsInitialized = true;
	}
}

CObjectClasses::~CObjectClasses()
{}

cv::Scalar CObjectClasses::getColor(const CyC_INT& _object_class)
{
	cv::Scalar color = 0;
	if (_object_class >= 0)
	{
		color = color::colormap[(_object_class < sizeof(color::colormap)) ? _object_class : 0];
	}
	else
	{
		switch (_object_class)
		{
			case CObjectClasses::UNDEFINED:
				color = color::undefined;
				break;
			case CObjectClasses::LIDAR:
				color = color::lidar;
				break;
			case CObjectClasses::ULTRASONICS:
				color = color::ultrasonics;
				break;
			case CObjectClasses::DEPTH:
				color = color::depth;
				break;
			case CObjectClasses::SLAM_MAP_POINTS:
				color = color::slam_map_points;
				break;
			default:
				color = color::undefined;
				break;
		}
	}
	return color;
}