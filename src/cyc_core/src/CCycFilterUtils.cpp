// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CCycFilterUtils.h"

CCycFilterUtils::CCycFilterUtils()
{}

CCycFilterUtils::~CCycFilterUtils()
{}

CCycFilterBase* CCycFilterUtils::getStateFilter(const CycInputSources& _input_filters)
{
    CCycFilterBase* pStateFilter = nullptr;
    
    for (size_t i = 0; i < _input_filters.size(); ++i)
    {
        if (_input_filters[i].pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE") ||
            _input_filters[i].pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_SIMULATION_FILTER_TYPE") ||
            _input_filters[i].pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_DRONE_STATE_ESTIMATION_FILTER_TYPE") ||
            _input_filters[i].pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_DRONE_SIMULATION_FILTER_TYPE"))
        {
            pStateFilter = _input_filters[i].pCycFilter;
        }

    }

    return pStateFilter;
}

bool CCycFilterUtils::getPose(CCycFilterBase* _filter, CyC_TIME_UNIT& _io_timestamp, CPose& _out_pose)
{
    if (_filter == nullptr)
        return false;

    bool bReturn = false;

    // Update pose from filter data, depending on the filter type (vehicle or drone)
    const CyC_TIME_UNIT readTsPose = _filter->getTimestampStop();

    if (readTsPose > _io_timestamp)
    {
        _io_timestamp = readTsPose;

        CycState state;
        if (_filter->getData(state))
        {
            bReturn = CCycFilterUtils::state2pose(state, _filter->getFilterType(), _out_pose);
        }
    }

    return bReturn;
}

bool CCycFilterUtils::state2pose(const CycState& _state, const CyC_FILTER_TYPE& _filter_type, CPose& _out_pose)
{
    bool bReturn = false;

    if (_filter_type == CStringUtils::CyC_HashFunc("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE") ||
        _filter_type == CStringUtils::CyC_HashFunc("CyC_VEHICLE_SIMULATION_FILTER_TYPE"))
    {
        _out_pose.update(_state.x_hat(0), _state.x_hat(1), 0.f, 0.f, 0.f, _state.x_hat(3));
        bReturn = true;
    }
        
    if (_filter_type == CStringUtils::CyC_HashFunc("CyC_DRONE_STATE_ESTIMATION_FILTER_TYPE") ||
        _filter_type == CStringUtils::CyC_HashFunc("CyC_DRONE_SIMULATION_FILTER_TYPE"))
    {
        _out_pose.update(_state.x_hat(0), _state.x_hat(1), _state.x_hat(2), _state.x_hat(9), _state.x_hat(10), _state.x_hat(11));
        bReturn = true;
    }
        
    return bReturn;
}

bool CCycFilterUtils::str2key(const std::string& _str, CycDatablockKey& _out_key)
{
    bool bReturn = false;

    std::vector<std::string> str_parts;
    CStringUtils::splitstring(_str, "-", str_parts);

    if (str_parts.size() == 2)
    {
        if (CStringUtils::is_positive_int(str_parts[0]) && CStringUtils::is_positive_int(str_parts[1]))
        {
            _out_key.nCoreID = stoi(str_parts[0]);
            _out_key.nFilterID = stoi(str_parts[1]);

            bReturn = true;
        }
    }

    return bReturn;
}
