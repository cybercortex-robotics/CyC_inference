# Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

cmake_minimum_required(VERSION 3.10)

project(qtplot)

find_package(Qt5 COMPONENTS Widgets Core Gui PrintSupport REQUIRED)

set(QTPLOT_LIB qtplot)

if (MSVC)
    set(LIB_TYPE STATIC)
else ()
    set(LIB_TYPE SHARED)
endif ()

set(LIB_SOURCES qtplot.cpp qcustomplot.cpp qtplotsegment.cpp qtplotapp.cpp qtimageapp.cpp)
qt5_wrap_cpp(LIB_SOURCES 
    qcustomplot.h 
    qtplotsegment.h 
    qtplotapp.h 
    qtimageapp.h)
add_library(${QTPLOT_LIB} ${LIB_TYPE} ${LIB_SOURCES})

target_include_directories(${QTPLOT_LIB} 
    PUBLIC 
        ${CMAKE_SOURCE_DIR}/src/cyc_toolkit/src
        ${CyC_THIRD_PARTY}/spdlog-1.4.2/include)

target_link_libraries(${QTPLOT_LIB} 
    PUBLIC 
        Qt5::Widgets 
        Qt5::Core 
        Qt5::Gui 
        Qt5::PrintSupport)

install ( TARGETS qtplot
          LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin)

install ( DIRECTORY include DESTINATION .
          PATTERN CMakeLists.txt EXCLUDE )
