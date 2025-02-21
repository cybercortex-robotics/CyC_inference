// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CMathUtils_H
#define CMathUtils_H

#include "CCR_TYPES.h"
#include <random>
#include <vector>

class CMathUtils
{
public:
    CMathUtils();
    ~CMathUtils();

    template<typename T>
    static T clamp(T min, T val, T max)
    {
        if (val < min)
            return min;
        if (val > max)
            return max;

        return val;
    }
};

#endif // CMathUtils_H