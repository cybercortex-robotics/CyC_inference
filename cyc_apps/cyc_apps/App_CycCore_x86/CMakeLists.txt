# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

# Check for Raspberry Pi platform
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7l")
	set(RASPBERRY_PI TRUE)
endif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7l")

#include_directories(
#	${CyC_OPENCV_INCLUDE_PATH}
#	${CMAKE_SOURCE_DIR}/src
#	${CMAKE_SOURCE_DIR}/src/CyC_toolkit/src
#	${CMAKE_SOURCE_DIR}/3rdParty/qt-5.12.2/win-msvc141-x64/include)

#if (UNIX)
#    include_directories(${CMAKE_SOURCE_DIR}/3rdParty/opencv-4.3.0/include)
#    link_directories(${CMAKE_SOURCE_DIR}/3rdParty/opencv-4.3.0/linux-gcc-x64/lib)
#elseif (MSVC)
#	link_directories(
#			${CMAKE_SOURCE_DIR}/3rdParty/opencv-4.3.0/win-msvc141-x64/lib
#			${CMAKE_SOURCE_DIR}/3rdParty/boost-1.62.0/win-msvc140-x64/lib
#			${CMAKE_SOURCE_DIR}/3rdParty/cvplot/lib)
#elseif (RASPBERRY_PI)
#	link_directories(
#			${CMAKE_SOURCE_DIR}/3rdParty/cvplot/lib
#			${CMAKE_SOURCE_DIR}/3rdParty/libconfig/linux-gcc-arm-x86/lib
#			/home/pi/opencv/install/lib/cmake/opencv4)
#endif()

add_executable(App_CycCore
	main_core.cpp
	CPlotFilterOutput.h
	CPlotFilterOutput.cpp)

target_include_directories(App_CycCore 
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/cyc_core/src
        ${CyC_THIRD_PARTY_INCLUDE_DIRECTORIES})

target_link_directories(App_CycCore 
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

if (MSVC)
	file(GLOB OCTOMAP_LIBRARIES "${CMAKE_SOURCE_DIR}/3rdParty/octomap/${CyC_LIBS_BINARY_KEY}/lib/*.lib")
	file(GLOB GLUT_LIBRARIES "${CMAKE_SOURCE_DIR}/3rdParty/glut-2.0.0.29/${CyC_LIBS_BINARY_KEY}/lib/*.lib")
    #link_directories(${CMAKE_SOURCE_DIR}/3rdParty/pthreads/${CyC_LIBS_BINARY_KEY}/lib)
	file(GLOB PTHREAD_LIBRARIES "${CMAKE_SOURCE_DIR}/3rdParty/pthreads/${CyC_LIBS_BINARY_KEY}/lib/*.lib")

    target_link_libraries(App_CycCore
        CycCore
        ${PTHREAD_LIBRARIES}
        octomap
        qtplot
        ${CyC_OPENCV_WORLD}
        ${OCTOMAP_LIBRARIES} 
        ${GLUT_LIBRARIES}
        pdh)
elseif (UNIX)
    target_link_libraries(App_CycCore
        CycCore
        octomap
        ${CyC_OPENCV_WORLD}
        qtplot
        pthread
        dl)
elseif(RASPBERRY_PI)
    target_link_libraries(App_CycCore
        CycCore
        ${CyC_OPENCV_WORLD}
        pthread
        pigpio)
endif ()

if (Build_Robot_ARParotDrone)
    add_compile_definitions(-DWITH_ARPAROTDRONE)
    target_link_libraries(App_CycCore
        CycCore
        Robot_ARParotDrone)
endif (Build_Robot_ARParotDrone)

# Copy App_CycCore.exe to App_CycCoreTwin.exe
if (MSVC)
    add_custom_command(TARGET App_CycCore POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/bin/App_CycCore.exe
        ${CMAKE_SOURCE_DIR}/bin/App_CycCoreTwin.exe)
else ()
    add_custom_command(TARGET App_CycCore POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/bin/App_CycCore
        ${CMAKE_SOURCE_DIR}/bin/App_CycCoreTwin)
endif ()

add_custom_target(BUILD_APP_WITH_FILTERS
    DEPENDS App_CycCore BUILD_ALL_FILTERS
)