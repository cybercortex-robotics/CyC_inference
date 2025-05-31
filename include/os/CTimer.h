// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CTIMER_H
#define CTIMER_H

#include "CyC_TYPES.h"

/**
 * This class is used to time some codes (in seconds).
 * On Unix, the resolution is up to microseconds (see gettimeofday()).
 * On Windows, the performance counter is used (see QueryPerformanceCounter() and QueryPerformanceFrequency()).
 * Example:
 * @code
 *      CTimer timer;
 *      timer.start();
 *      ... (do some work)
 *      timer.stop();
 *      double seconds = timer.getInterval();
 *      ...
 * @endcode
 */
class CTimer
{
public:
	CTimer();
    ~CTimer();

    /**
     * This method is used to get
     * the time of the system right now.
     * @return double the time in seconds.
     */
    static CyC_TIME_UNIT now();

    /**
     * This method starts the timer.
     */
    void start();

    /**
     * This method stops the timer.
     */
    void stop();

    /**
     * These 2 methods are used to get the elapsed time
     * between now and the start(). If timer is stopped, the interval time
	 * between stop() and the start() is returned.
     * @return CyC_TIME_UNIT the interval in milliseconds/microseconds.
     */
    CyC_TIME_UNIT elapsedMilliseconds() { return getElapsedTimeMilliseconds(); }
    CyC_TIME_UNIT elapsedMicroseconds() { return getElapsedTimeMicroseconds(); }

    CyC_TIME_UNIT getElapsedTimeMilliseconds();
    CyC_TIME_UNIT getElapsedTimeMicroseconds();

    /**
     * This method is used to get the interval of
     * the timer while it is running. It's automatically
     * stop the timer, get the interval and restart
     * the timer. It's the same of calling stop(),
     * elapsed() and start(). Method restart() does the same thing, for convenience.
     * @return double the interval in seconds.
     */
    CyC_TIME_UNIT restart()
	{
		return ticks();
	}

    CyC_TIME_UNIT ticks();

    static std::string toString(CyC_TIME_UNIT ts);

private:
	using clock_type_t = std::chrono::high_resolution_clock;
	using time_point_t = std::chrono::time_point<clock_type_t>;
	using time_precision_milliseconds_t = std::chrono::milliseconds;
    using time_precision_microsesonds_t = std::chrono::microseconds;
	static constexpr int32_t PRECISION_MAGNITUDE = 1000; // milliseconds to seconds

	time_point_t m_start_time;
	time_point_t m_stop_time;
};

#endif //CTIMER_H
