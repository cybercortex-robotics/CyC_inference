# Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

find_package (Qt5Core REQUIRED)

if (MSVC)
    set(LIB_TYPE STATIC)
else ()
    set(LIB_TYPE SHARED)
endif ()

add_library(CycCore ${LIB_TYPE}
    src/CyC_TYPES.h
    src/CCycFilterBase.h
    src/CCycFilterBase.cpp
    src/CConfigCheck.cpp
    src/CConfigCheck.h
    src/CCycCache.h
    src/CConfigParameters.h
    src/CConfigParameters.cpp
    src/CCycDatablock.h
    src/CCycDatablock.cpp
    src/CCycCore.h
    src/CCycCore.cpp
    src/CHddStorage.h
    src/CHddStorage.cpp
    src/CCycSyncReader.h
    src/CCycSyncReader.cpp
    src/CCycFilterUtils.h
    src/CCycFilterUtils.cpp
    src/nonstd/any.hpp
    src/dynalo/dynalo.hpp
    src/dynalo/symbol_helper.hpp
    src/dynalo/detail/config.hpp
    src/dynalo/detail/linux/dynalo.hpp
    src/dynalo/detail/windows/dynalo.hpp
    src/dynalo/detail/macos/dynalo.hpp)

target_include_directories(CycCore
    PUBLIC
        ${Qt5Core_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/src/cyc_core/src
        ${CMAKE_SOURCE_DIR}/src/cyc_toolkit/src
        ${CyC_THIRD_PARTY_INCLUDE_DIRECTORIES})

target_link_directories(CycCore
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

target_link_libraries(CycCore
    PUBLIC
        CycToolkit
        qtplot
        octomap
        octomath
        ${CyC_OPENCV_WORLD})

if (MSVC)
    target_compile_definitions(CycCore
        PUBLIC
            _SILENCE_STDEXT_ARR_ITERS_DEPRECATION_WARNING)

    target_link_libraries(CycCore
        PUBLIC
            pdh
            libconfig++
            ws2_32
            winmm)
elseif (RASPBERRY_PI)
    target_link_libraries(CycCore
        PUBLIC
            stdc++fs
            atomic)
elseif (ANDROID)
	target_link_libraries(CycCore
		PRIVATE
			config++)
endif ()

if (ANDROID)
	add_library(CcrVisionJNI SHARED
		CyC_core/jni/common.h
		CyC_core/jni/CCycCore.cpp
		CyC_core/jni/CBaseCcrFilter.cpp
		CyC_core/jni/CDataGrabber.cpp
		CyC_core/jni/CRemoteCoreHelper.cpp)
		
#	add_library(CcrCvPlot SHARED	
		#../3rdParty/cvplot-0.0.3/src/src/cvplot_android/cvplot.cc	
		#../3rdParty/cvplot-0.0.3/src/src/cvplot_android/color.cc	
		#../3rdParty/cvplot-0.0.3/src/src/cvplot_android/figure.cc	
		#../3rdParty/cvplot-0.0.3/src/src/cvplot_android/highgui.cc	
		#../3rdParty/cvplot-0.0.3/src/src/cvplot_android/window.cc	
		#../3rdParty/cvplot-0.0.3/src/src/cvplot_android/internal.h)	

	#target_link_libraries(CcrCvPlot	
	#	PRIVATE	
	#	opencv_java4)

	target_link_libraries(CycCoreJni
		PRIVATE
			CycCore
			cvplot
			opencv_java4
			log
			GLESv2
			android
			camera2ndk
			mediandk)
endif (ANDROID)
