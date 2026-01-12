// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/*
 * main_ccr_daemon_linux.cpp
 *
 *  Created on: 24.10.2021
 *      Author: Sorin Grigorescu
 *
 * Linux daemon for managing a ccr core
 * 
 * https://github.com/pasce/daemon-skeleton-linux-c
 * https://stackoverflow.com/questions/30850759/how-to-start-process-on-linux-os-in-c-c#:~:text=Use%20fork()%20system%20call,the%20path%20of%20the%20executable.
 * https://unix.stackexchange.com/questions/56957/how-to-start-an-application-automatically-on-boot
 * https://medium.com/@benmorel/creating-a-linux-service-with-systemd-611b5c8b91d6
 * https://blog.christophersmart.com/2022/10/20/making-a-systemd-service-which-binds-to-an-ip-start-on-boot/
 * https://askubuntu.com/questions/65563/how-do-i-set-an-environment-variable-at-boot-time-via-a-script
 */

#include <CyC_TYPES.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <dirent.h>
#include <iterator>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <syslog.h>
#include <unistd.h>
#include <vector>
#include <signal.h>

// For security purposes, we don't allow any arguments to be passed into the daemon
int main(void)
{
    // Define variables
    pid_t pid, sid;

    std::cout << "Parent pid = " << getpid() << std::endl;

    // Fork the current process
    pid = fork();

    if(pid > 0)
    {
        // The parent process continues with a process ID greater than 0
        std::cout << "The parent process continues with a process ID greater than 0, which is = " << getpid() << std::endl;
        std::cout << "child = " << pid << std::endl;

        // Wait for 6 seconds, then kill the child process
        std::this_thread::sleep_for(std::chrono::milliseconds(6000));
        int ret = kill(pid, SIGTERM);
        std::cout << "child killed = " << ret << std::endl;

        while (1)
        {}

        // Wait for another 6 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(6000));

        return EXIT_SUCCESS;
    }
    else if (pid < 0)
    {
        // A process ID lower than 0 indicates a failure in either process
        return EXIT_FAILURE;
    }
    else
    {
        // Executed by child (preparation work)

        // Change the current working directory to a directory guaranteed to exist
        std::string path = "/home/sorin/dev/src/CyberCortex.AI/core/bin";
        if ((chdir(path.c_str())) < 0)
        {
            // Log failure and exit
            std::cout << "Could not change working directory to " << path << std::endl;

            // If our guaranteed directory does not exist, terminate the child process to ensure the daemon has not been hijacked
            return EXIT_FAILURE;
        }

        std::cout << "child pid: " << getpid() << std::endl;
        //int ret = std::system("./App_CycCoreTwin ../etc/pipelines/daemons/daemon_carla_controller.conf");

        
        std::cout << "END pid == 0" << std::endl;
    }
    // The parent process has now terminated, and the forked child process will continue
    // (the pid of the child process is now 0)

    std::cout << "CURRENT process pid = " << getpid() << std::endl; // <- this is now the pid of the child
    std::cout << "child pid = " << pid << std::endl; // <- this is now 0

    // Since the child process is a daemon, the umask needs to be set so files and logs can be written
    //umask(0);

    // Generate a session ID for the child process
    sid = setsid();
    
    // Ensure a valid SID for the child process
    if(sid < 0)
    {
        // Log failure and exit
        std::cout << "Could not generate session ID for child process" << std::endl;

        // If a new session ID could not be generated, we must terminate the child process or it will be orphaned
        return EXIT_FAILURE;
    }

    // A daemon cannot use the terminal, so close standard file descriptors for security reasons
    // close(STDIN_FILENO);
    // close(STDOUT_FILENO);
    // close(STDERR_FILENO);

    // Daemon-specific intialization should go here
    const int SLEEP_INTERVAL = 1000;
    
    // Sleep for a period of time
    //std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_INTERVAL));

    while (1)
    {}
    
    std::cout << "CHILD END - this will never be reached" << std::endl;

    // Terminate the child process when the daemon completes
    return EXIT_SUCCESS;
}
