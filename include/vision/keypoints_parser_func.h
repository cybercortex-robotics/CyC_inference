// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <vector>
#include <string>
#include "CCR_TYPES.h"
#include "csv_reader.h"

static constexpr size_t expected_cols = 7;

static void get_common_keypoint_ids_from_csv(
    const std::string& filepath,
    const std::vector<std::string>& imgs,
    std::vector<size_t>& ids_out
);

/* Returns 0 on success, -1 if the file couldn't be open
or -2 if the CSV file was in any way invalid. */
int get_keypoints_from_csv(
    const std::string& filepath,
    const std::vector<std::string>& imgs,
    std::vector<std::vector<Eigen::Vector2f>>& pts
);