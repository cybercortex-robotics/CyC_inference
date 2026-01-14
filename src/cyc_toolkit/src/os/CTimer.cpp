// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CTimer.h"

///////////////////////
// public:
///////////////////////
CTimer::CTimer()
{
}

CTimer::~CTimer()
{
}

CyC_TIME_UNIT CTimer::now()
{
    //const auto epoch_time = clock_type_t::now().time_since_epoch();
    //return std::chrono::duration_cast<time_precision_milliseconds_t>(epoch_time).count();
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void CTimer::start()
{
    m_start_time = clock_type_t::now();
}

void CTimer::stop()
{
    m_stop_time = clock_type_t::now();
}

CyC_TIME_UNIT CTimer::getElapsedTimeMilliseconds()
{
	const auto elapsed_time = clock_type_t::now() - m_start_time;
    return std::chrono::duration_cast<time_precision_milliseconds_t>(elapsed_time).count();
}

CyC_TIME_UNIT CTimer::getElapsedTimeMicroseconds()
{
    const auto elapsed_time = clock_type_t::now() - m_start_time;
    return std::chrono::duration_cast<time_precision_microsesonds_t>(elapsed_time).count();
}

CyC_TIME_UNIT CTimer::ticks()        // Stop->start and return Interval
{
    auto inter = elapsedMilliseconds();
    start();
    return inter;
}

std::string CTimer::toString(CyC_TIME_UNIT ts)
{
    std::chrono::milliseconds dur(ts);
    std::chrono::time_point<std::chrono::system_clock> dt(dur);
    //time_point_t dt(dur);
    std::time_t t = std::chrono::system_clock::to_time_t(dt);
    //std::time_t t = clock_type_t::to_time_t(dt);

    std::string str = std::ctime(&t);
    
    return str;
}
