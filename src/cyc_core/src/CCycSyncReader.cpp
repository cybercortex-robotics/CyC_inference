// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CCycSyncReader.h"
#include "CCycFilterBase.h"

void CCycSyncReader::setFilters(const std::vector<CCycFilterBase*>& filters)
{
    m_filters = filters;

    m_lastTimestamps.clear();
    m_lastTimestamps.resize(m_filters.size());
    std::fill(m_lastTimestamps.begin(), m_lastTimestamps.end(), -1);
}

bool CCycSyncReader::dataIsReady() const
{
    bool dataReady = true;
    for (size_t i = 0; i < m_filters.size(); ++i)
    {
        dataReady &= m_filters[i]->getTimestampStop() > m_lastTimestamps[i];
    }

    return dataReady;
}
