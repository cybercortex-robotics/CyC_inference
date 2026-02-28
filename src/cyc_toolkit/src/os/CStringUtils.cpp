// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CStringUtils.h"
#include <sstream>

CStringUtils::CStringUtils()
{}

CStringUtils::~CStringUtils()
{}

void CStringUtils::splitstring(const std::string& str, const std::string& delim, std::vector<std::string>& parts)
{
    size_t start, end = 0;
    while (end < str.size()) {
        start = end;
        while (start < str.size() && (delim.find(str[start]) != std::string::npos)) {
            start++;  // skip initial whitespace
        }
        end = start;
        while (end < str.size() && (delim.find(str[end]) == std::string::npos)) {
            end++; // skip to end of word
        }
        if (end - start != 0) {  // just ignore zero-length strings.
            parts.push_back(std::string(str, start, end - start));
        }
    }
}

void CStringUtils::copy(const std::string& from, const std::string& to)
{
    std::ifstream src(from.c_str());
    std::ofstream dst(to.c_str());

    dst << src.rdbuf();
}

void CStringUtils::stringToBool(const std::string& sParam, bool& val)
{
    bool bConversion(true);
    if (!sParam.empty())
    {
        std::string sLoweredParam;
        sLoweredParam.resize(sParam.size());
        std::transform(sParam.begin(), sParam.end(), sLoweredParam.begin(), ::tolower);

        bConversion = sLoweredParam == "true";
    }

    val = bConversion;
}

std::string CStringUtils::remove_spaces(const std::string& _s)
{
    std::string s = _s;
    s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) {
        return std::isspace(c); // Remove spaces and other whitespace
        }), s.end());
    return s;
}

bool CStringUtils::is_positive_int(const std::string& _s)
{
    std::string s = remove_spaces(_s);
    return !s.empty() && std::find_if(s.begin(),
        s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
}

bool CStringUtils::is_float(const std::string& _s)
{
    std::istringstream iss(_s);
    float f;
    iss >> std::noskipws >> f; // noskipws considers leading whitespace invalid
    // Check the entire string was consumed and if either failbit or badbit is set
    return iss.eof() && !iss.fail();
}

std::string CStringUtils::bool2str(const bool& _b)
{
    if (_b)
        return "TRUE";
    else
        return "FALSE";
}

size_t CStringUtils::CyC_HashFunc(const std::string& str)
{
    constexpr size_t FNV_OFFSET_BASIS = 14695981039346656037ULL;
    constexpr size_t FNV_PRIME = 1099511628211ULL;
    size_t hash = FNV_OFFSET_BASIS;
    for (char c : str)
    {
        hash ^= static_cast<size_t>(c);
        hash *= FNV_PRIME;
    }
    return hash;
}

std::string CStringUtils::padding_int2str(int _value, int _padding)
{
    std::string s = std::to_string(_value);
    int currentLen = static_cast<int>(s.length());
    if (currentLen < _padding)
        s.insert(0, _padding - currentLen, '0');
    return s;
}
