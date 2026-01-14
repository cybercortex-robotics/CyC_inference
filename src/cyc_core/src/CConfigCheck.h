// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#pragma once

#include "CyC_TYPES.h"

namespace CConfigCheck
{
    bool check(const std::string &configfile);
    bool exists(const std::string &configfile);
}
