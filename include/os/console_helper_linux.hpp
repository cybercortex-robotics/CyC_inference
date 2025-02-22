// Copyright (c) 2024 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CONSOLE_HELPER_LINUX_H_
#define CONSOLE_HELPER_LINUX_H_

#include <sys/ioctl.h>
#include <termios.h>

void cls()
{
}

void setup()
{
}

void restore()
{
    // Reset colors
    printf("\x1b[0m");
}

void enable_raw_mode()
{
    termios term;
    tcgetattr(0, &term);
    term.c_lflag &= ~(ICANON | ECHO); // Disable echo as well
    tcsetattr(0, TCSANOW, &term);
}

void disable_raw_mode()
{
    termios term;
    tcgetattr(0, &term);
    term.c_lflag |= ICANON | ECHO;
    tcsetattr(0, TCSANOW, &term);
}

bool kbhit()
{
    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);
    return byteswaiting > 0;
}

int gc()
{
    return getchar();
}

class cursor_guard
{
public:
    cursor_guard()
    {
    }

    cursor_guard(const cursor_guard&) = delete;
    cursor_guard(cursor_guard&&) = delete;
    cursor_guard& operator=(const cursor_guard&) = delete;
    cursor_guard& operator=(cursor_guard&&) = delete;

    ~cursor_guard()
    {
        printf("\033[2J");
    }
};

#endif // CONSOLE_HELPER_LINUX_H_
