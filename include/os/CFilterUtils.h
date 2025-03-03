// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CFilterUtils_H_
#define CFilterUtils_H_

#include "CCR_TYPES.h"
#include "CBaseCcrFilter.h"

class CFilterUtils
{
public:
	CFilterUtils();
	~CFilterUtils();

	static CBaseCcrFilter* getStateFilter(const CcrInputSources& _input_filters);
	static bool getPose(CBaseCcrFilter* _filter, CCR_TIME_UNIT& _io_timestamp, CPose& _out_pose);
	static bool state2pose(const CcrState& _state, const CCR_FILTER_TYPE& _filter_type, CPose& _out_pose);
	static bool str2key(const std::string& _str, CcrDatablockKey& _out_key);
};

#endif /* CFilterUtils_H_ */
