// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CSTRINGUTILS_H_
#define CSTRINGUTILS_H_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>

class CStringUtils
{
public:
	CStringUtils();
	virtual ~CStringUtils();

	static void splitstring(const std::string& str, const std::string& delim, std::vector<std::string>& parts);

	static void copy(const std::string& from, const std::string& to);

	static void stringToBool(const std::string& sParam, bool& val);

	static std::string remove_spaces(const std::string& _s);

	static bool is_positive_int(const std::string& _s);

	static bool is_float(const std::string& _s);
};
#endif /* CSTRINGUTILS_H_ */
