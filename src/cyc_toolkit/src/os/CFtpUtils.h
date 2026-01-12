// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CFtpUtils_H
#define CFtpUtils_H

#include "CyC_TYPES.h"

class CFtpUtils
{
public:
    static bool upload_file(const std::string& _url,
        const std::string& _local_file_path,
        const std::string& _credentials);
};
#endif /* CFtpUtils_H */
