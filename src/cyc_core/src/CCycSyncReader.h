// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CCycSyncReader_H_
#define CCycSyncReader_H_

#include <CyC_TYPES.h>
#include "CCycFilterBase.h"

class CCycSyncReader
{
public:
    CCycSyncReader() = default;
    CCycSyncReader(const CCycSyncReader&) = default;
    CCycSyncReader(CCycSyncReader&&) = default;
    CCycSyncReader& operator=(const CCycSyncReader&) = default;
    CCycSyncReader& operator=(CCycSyncReader&&) = default;
    ~CCycSyncReader() = default;

    void setFilters(const std::vector<CCycFilterBase*>& filters);
    bool dataIsReady() const;

    template <typename Arg, typename... Args>
    bool getData(Arg& arg, Args&... args);

private:
    template <typename Arg, typename... Args>
    bool getDataImpl(Arg& arg, Args&... args);

    template <typename Arg>
    bool getDataImpl(Arg& arg);

    std::vector<CCycFilterBase*> m_filters;
    std::vector<CyC_TIME_UNIT> m_lastTimestamps;

    size_t m_readIndex = 0;
};

template <typename Arg, typename... Args>
bool CCycSyncReader::getData(Arg& arg, Args&... args)
{
    if (m_filters.empty())
    {
        return false;
    }

    if ((sizeof...(args) + 1) != m_filters.size())
    {
        spdlog::error("CCycSyncReader: Number of arguments does not match the number of filters.");
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
bool CCycSyncReader::getDataImpl(Arg& arg, Args&... args)
{
    if (!getDataImpl(arg))
    {
        return false;
    }

    return getDataImpl(args...);
}

template <typename Arg>
bool CCycSyncReader::getDataImpl(Arg& arg)
{
    m_lastTimestamps[m_readIndex] = m_filters[m_readIndex]->getTimestampStop();
    const auto result = m_filters[m_readIndex]->getData(arg);
    ++m_readIndex;

    return result;
}


#endif // CCycSyncReader_H_
