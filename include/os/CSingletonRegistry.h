// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSingletonRegistry_H_
#define CSingletonRegistry_H_

#include "CSingletonBase.h"
#include <unordered_map>

class CSingletonRegistry
{
public:
    CSingletonRegistry() = default;
    CSingletonRegistry(const CSingletonRegistry&) = delete;
    CSingletonRegistry(CSingletonRegistry&&) = delete;
    CSingletonRegistry& operator=(const CSingletonRegistry&) = delete;
    CSingletonRegistry& operator=(CSingletonRegistry&&) = delete;
    ~CSingletonRegistry() = default;

    template <typename T, typename... Args>
    void registerInstance(Args&&... args)
    {
        m_Registry[T::getType()] = T::create_instance(std::forward<Args>(args)...);
    }

    template <typename T>
    std::shared_ptr<T> get()
    {
        if (m_Registry.count(T::getType()))
            return std::dynamic_pointer_cast<T>(m_Registry.at(T::getType()));
        else
            return nullptr;
    }

private:
    std::unordered_map<std::string, std::shared_ptr<CSingletonBase>> m_Registry;
};

#endif /* CSingletonRegistry_H_ */
