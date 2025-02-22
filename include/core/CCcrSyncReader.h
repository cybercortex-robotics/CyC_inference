// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CCcrSyncReader_H_
#define CCcrSyncReader_H_

#include <CCR_TYPES.h>
#include "core/CBaseCcrFilter.h"

class CCcrSyncReader
{
public:
    CCcrSyncReader() = default;
    CCcrSyncReader(const CCcrSyncReader&) = default;
    CCcrSyncReader(CCcrSyncReader&&) = default;
    CCcrSyncReader& operator=(const CCcrSyncReader&) = default;
    CCcrSyncReader& operator=(CCcrSyncReader&&) = default;
    ~CCcrSyncReader() = default;

    void setFilters(const std::vector<CBaseCcrFilter*>& filters);
    bool dataIsReady() const;

    template <typename Arg, typename... Args>
    bool getData(Arg& arg, Args&... args);

private:
    template <typename Arg, typename... Args>
    bool getDataImpl(Arg& arg, Args&... args);

    template <typename Arg>
    bool getDataImpl(Arg& arg);

    std::vector<CBaseCcrFilter*> m_filters;
    std::vector<CCR_TIME_UNIT> m_lastTimestamps;

    size_t m_readIndex = 0;
};

template <typename Arg, typename... Args>
bool CCcrSyncReader::getData(Arg& arg, Args&... args)
{
    if (m_filters.empty())
    {
        return false;
    }

    if ((sizeof...(args) + 1) != m_filters.size())
    {
        spdlog::error("CCcrSyncReader: Number of arguments does not match the number of filters.");
        return false;
    }

    if (!dataIsReady())
    {
        return false;
    }

    m_readIndex = 0;
    return getDataImpl(arg, args...);
}

template <typename Arg, typename... Args>
bool CCcrSyncReader::getDataImpl(Arg& arg, Args&... args)
{
    if (!getDataImpl(arg))
    {
        return false;
    }

    return getDataImpl(args...);
}

template <typename Arg>
bool CCcrSyncReader::getDataImpl(Arg& arg)
{
    m_lastTimestamps[m_readIndex] = m_filters[m_readIndex]->getTimestampStop();
    const auto result = m_filters[m_readIndex]->getData(arg);
    ++m_readIndex;

    return result;
}


#endif // CCcrSyncReader_H_
