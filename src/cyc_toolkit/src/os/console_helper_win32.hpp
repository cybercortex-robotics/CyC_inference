// Copyright (c) 2024 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CONSOLE_HELPER_WIN32_
#define CONSOLE_HELPER_WIN32_

#include <windows.h>
#include <conio.h>

#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING  0x0004
#endif

static HANDLE stdoutHandle;
static DWORD outModeInit;

void cls()
{
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

    COORD coordScreen = { 0, 0 };    // home for the cursor
    DWORD cCharsWritten;
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    DWORD dwConSize;

    // Get the number of character cells in the current buffer.
    if (!GetConsoleScreenBufferInfo(hConsole, &csbi))
    {
        return;
    }

    dwConSize = csbi.dwSize.X * csbi.dwSize.Y;

    // Fill the entire screen with blanks.
    if (!FillConsoleOutputCharacter(hConsole,        // Handle to console screen buffer
        (TCHAR)' ',      // Character to write to the buffer
        dwConSize,       // Number of cells to write
        coordScreen,     // Coordinates of first cell
        &cCharsWritten)) // Receive number of characters written
    {
        return;
    }

    // Get the current text attribute.
    if (!GetConsoleScreenBufferInfo(hConsole, &csbi))
    {
        return;
    }

    // Set the buffer's attributes accordingly.
    if (!FillConsoleOutputAttribute(hConsole,         // Handle to console screen buffer
        csbi.wAttributes, // Character attributes to use
        dwConSize,        // Number of cells to set attribute
        coordScreen,      // Coordinates of first cell
        &cCharsWritten))  // Receive number of characters written
    {
        return;
    }

    // Put the cursor at its home coordinates.
    SetConsoleCursorPosition(hConsole, coordScreen);
}

void setup()
{
    DWORD outMode = 0;
    stdoutHandle = GetStdHandle(STD_OUTPUT_HANDLE);

    if(stdoutHandle == INVALID_HANDLE_VALUE)
    {
        exit(GetLastError());
    }

    if(!GetConsoleMode(stdoutHandle, &outMode))
    {
        exit(GetLastError());
    }

    outModeInit = outMode;

    // Enable ANSI escape codes
    outMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;

    if(!SetConsoleMode(stdoutHandle, outMode))
    {
        //exit(GetLastError());
    }
}

void restore()
{
    // Reset colors
    printf("\x1b[0m");

    // Reset console mode
    if(!SetConsoleMode(stdoutHandle, outModeInit))
    {
        exit(GetLastError());
    }
}

void enable_raw_mode()
{
}

void disable_raw_mode()
{
}

//bool kbhit()
//{
//    return _kbhit();
//}

int gc()
{
    return getchar();
}

class cursor_guard
{
    HANDLE hConsole;
    CONSOLE_SCREEN_BUFFER_INFO csbi;

public:
    cursor_guard()
        : csbi{0}
    {
        hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
        if (hConsole)
        {
            GetConsoleScreenBufferInfo(hConsole, &csbi);
        }
    }

    cursor_guard(const cursor_guard&) = delete;
    cursor_guard(cursor_guard&&) = delete;
    cursor_guard& operator=(const cursor_guard&) = delete;
    cursor_guard& operator=(cursor_guard&&) = delete;

    ~cursor_guard()
    {
        if (hConsole)
        {
            SetConsoleCursorPosition(hConsole, csbi.dwCursorPosition);
        }
    }
};


#endif // CONSOLE_HELPER_WIN32_
