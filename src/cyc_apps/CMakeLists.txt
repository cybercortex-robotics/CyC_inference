# Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(Build_App_CycCore TRUE CACHE BOOL "Build_App_CycCore")
set(Build_App_CycDaemon FALSE CACHE BOOL "Build_App_CycDaemon")
set(Build_App_SignalingServer FALSE CACHE BOOL "Build_App_SignalingServer")

if (Build_App_CycCore)
    add_subdirectory(App_CycCore_x86)
endif (Build_App_CycCore)

if (Build_App_CycDaemon)
    add_subdirectory(App_CycDaemon)
endif (Build_App_CycDaemon)

if (Build_App_SignalingServer)
    add_subdirectory(App_SignalingServer)
endif (Build_App_SignalingServer)