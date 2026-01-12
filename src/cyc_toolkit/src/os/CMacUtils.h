// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CMacUtils_H
#define CMacUtils_H

#include "CyC_TYPES.h"
#include <stdio.h>

class CMacUtils
{
public:
    static CyC_ULONG getMAC();

private:
    static CyC_ULONG getMAC_Windows();
    static CyC_ULONG getMAC_Unix();
    static CyC_ULONG convert_mac_2_uint(std::string mac);
};
#endif /* CMacUtils_H */
