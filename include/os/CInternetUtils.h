// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CInternetUtils_H
#define CInternetUtils_H

#include "CCR_TYPES.h"
#include <stdio.h>

class CInternetUtils
{
public:
    static CCR_ULONG getMAC();

private:
    static CCR_ULONG getMAC_Windows();
    static CCR_ULONG getMAC_Unix();
    static CCR_ULONG convert_mac_2_uint(std::string mac);
};
#endif /* CInternetUtils_H */
