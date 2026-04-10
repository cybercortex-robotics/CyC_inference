// Copyright (c) 2024 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CONSOLE_HELPER_H_
#define CONSOLE_HELPER_H_

#include <iostream>
#ifdef WIN32
// Include system headers before the namespace to avoid pulling POSIX names into console::
#include <windows.h>
#include <conio.h>
#endif

namespace console
{

#ifdef WIN32
#include "console_helper_win32.hpp"
#else
#include "console_helper_linux.hpp"
#endif

}

#endif // CONSOLE_HELPER_H_
