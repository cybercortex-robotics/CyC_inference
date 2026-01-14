# Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

if (WIN32)
    file(GLOB SRC_ENTRY_POINT "main_cyc_daemon_win.cpp")
else ()
    file(GLOB SRC_ENTRY_POINT "main_cyc_daemon_linux.cpp")
endif ()

add_executable(App_CycDaemon ${SRC_ENTRY_POINT})

target_link_libraries(App_CycDaemon
    PUBLIC
		${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

target_link_libraries(App_CycDaemon
	PUBLIC
        CycCore
        CycToolkit)
