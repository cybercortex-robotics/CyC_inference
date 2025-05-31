// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CFilterUtils_H_
#define CFilterUtils_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"

class CFilterUtils
{
public:
	CFilterUtils();
	~CFilterUtils();

	static CCycFilterBase* getStateFilter(const CycInputSources& _input_filters);
	static bool getPose(CCycFilterBase* _filter, CyC_TIME_UNIT& _io_timestamp, CPose& _out_pose);
	static bool state2pose(const CycState& _state, const CyC_FILTER_TYPE& _filter_type, CPose& _out_pose);
	static bool str2key(const std::string& _str, CycDatablockKey& _out_key);
};

#endif /* CFilterUtils_H_ */
