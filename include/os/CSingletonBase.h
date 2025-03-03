// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSingletonBase_H_
#define CSingletonBase_H_

class CSingletonBase
{
public:
    CSingletonBase() = default;
    CSingletonBase(const CSingletonBase&) = delete;
    CSingletonBase(CSingletonBase&&) = delete;
    CSingletonBase& operator=(const CSingletonBase&) = delete;
    CSingletonBase& operator=(CSingletonBase&&) = delete;
    virtual ~CSingletonBase() = default;
};

#endif /* CSingletonBase_H_ */
