// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CCONVERSIONS_H_
#define CCONVERSIONS_H_

#include "CCR_TYPES.h"

class CConversions
{
public:
	CConversions();
	virtual ~CConversions();

    static std::string DataType2String(CCR_DATA_TYPE nDataType);
	static CCR_DATA_TYPE String2DataType(const std::string& sDataType);
};

#endif /* CCONVERSIONS_H_ */
