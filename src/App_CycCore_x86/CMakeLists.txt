# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

# Check for Raspberry Pi platform
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7l")
	set(RASPBERRY_PI TRUE)
endif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7l")

add_executable(App_CycCore
	src/main_core.cpp
	src/CPlotFilterOutput.h
	src/CPlotFilterOutput.cpp)

target_include_directories(App_CycCore 
    PUBLIC
        ${CyC_THIRD_PARTY_INCLUDE_DIRECTORIES})

target_link_directories(App_CycCore 
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

target_link_libraries(App_CycCore
    CycCore
    CycToolkit
    octomap
    octomath
    libconfig++
    libcsv
    ${CyC_OPENCV_WORLD})

if (MSVC)
	file(GLOB OCTOMAP_LIBRARIES "${CMAKE_SOURCE_DIR}/3rdParty/octomap/${CyC_LIBS_BINARY_KEY}/lib/*.lib")
	file(GLOB GLUT_LIBRARIES "${CMAKE_SOURCE_DIR}/3rdParty/glut-2.0.0.29/${CyC_LIBS_BINARY_KEY}/lib/*.lib")
    #link_directories(${CMAKE_SOURCE_DIR}/3rdParty/pthreads/${CyC_LIBS_BINARY_KEY}/lib)
	file(GLOB PTHREAD_LIBRARIES "${CMAKE_SOURCE_DIR}/3rdParty/pthreads/${CyC_LIBS_BINARY_KEY}/lib/*.lib")
    
    find_package (OpenGL REQUIRED)
    find_package (Qt5Core REQUIRED)
    find_package (Qt5Gui REQUIRED)
    find_package (Qt5OpenGL REQUIRED)
    find_package (Qt5Widgets REQUIRED)
    find_package (Qt5Xml REQUIRED)
    find_package (Qt5Network REQUIRED)
    find_package (Qt5Concurrent REQUIRED)
    find_package (Qt5PrintSupport REQUIRED)

    target_link_libraries(App_CycCore
        qtplot
        pdh
        Winmm
        Qt5::Concurrent
        Qt5::Core
        Qt5::Gui
        Qt5::Network
        Qt5::OpenGL
        Qt5::Widgets
        Qt5::Xml
        Qt5::PrintSupport
        ${PTHREAD_LIBRARIES}
        ${OCTOMAP_LIBRARIES} 
        ${GLUT_LIBRARIES})
elseif (UNIX)
    target_link_libraries(App_CycCore
        qtplot
        pthread
        dl)
elseif(RASPBERRY_PI)
    target_link_libraries(App_CycCore
        pthread
        pigpio)
endif ()

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