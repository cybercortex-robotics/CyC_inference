# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

######## USER DEFINED VARIABLES ########
# Define the OpenCV version
set(CCR_OPENCV_VERSION "4.9.0")

# Define the CUDA Toolkit SDK version
set(CCR_CUDA_VERSION "11.8")

# Define the libtorch and torchvision versions
set(CCR_TORCH_VERSION "2.1.2")
set(CCR_ONNXRUNTIME_VERSION "1.16.3")

# Define the Ceres version
set(CCR_CERES_VERSION "2.1.0")
######## !USER DEFINED VARIABLES ########


set(CCR_OPENCV_WORLD "opencv_world")
set(CCR_LIBCONFIG "config++")

# Check if CUDA version is given
if (CCR_CUDA_VERSION STREQUAL "")
  set(CCR_CUDA_VERSION_ID "nocuda")
else()
  string(REPLACE "." "" CCR_CUDA_VERSION_ID ${CCR_CUDA_VERSION})
  set(CCR_CUDA_VERSION_ID "cuda${CCR_CUDA_VERSION_ID}")
endif()

string(REPLACE "." "" CCR_OPENCV_VERSION_ID ${CCR_OPENCV_VERSION})

set(Build_UnitTests FALSE CACHE BOOL "Build_UnitTests")
set(Build_Examples FALSE CACHE BOOL "Build_Examples")

# Define target chipset
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CCR_CHIPSET_VERSION "x64")
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "AMD64")
    set(CCR_CHIPSET_VERSION "x64")
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "IA64")
    set(CCR_CHIPSET_VERSION "x64")
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")  # ARM x64 chipset (NVIDIA Jetson AGX)
	set(CCR_CHIPSET_VERSION "arm-x64")
    set(GENIUS_BOARD TRUE)
	add_definitions(-DGENIUS_BOARD)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7l")   # ARM x86 chipset (Raspberry PI)
	set(CCR_CHIPSET_VERSION "arm-x86")
    set(RASPBERRY_PI TRUE)
	add_definitions(-DRASPBERRY_PI)
endif ()

# Define target operating system
if (WIN32)
    set(CCR_OS_VERSION "win")
    set(CCR_OS_VERSION ${CCR_OS_VERSION} PARENT_SCOPE)
	set(CCR_OPENCV_WORLD "opencv_world${CCR_OPENCV_VERSION_ID}")
    set(CCR_OPENCV_WORLD ${CCR_OPENCV_WORLD} PARENT_SCOPE)
	set(CCR_LIBCONFIG "libconfig++")
    set(CCR_LIBCONFIG ${CCR_LIBCONFIG} PARENT_SCOPE)
elseif (UNIX)
    set(CCR_OS_VERSION "linux")
    set(CCR_OS_VERSION ${CCR_OS_VERSION} PARENT_SCOPE)
elseif (ANDROID)
	set(CCR_OS_VERSION "android")
    set(CCR_OS_VERSION ${CCR_OS_VERSION} PARENT_SCOPE)
endif ()

if (CMAKE_SYSTEM_NAME STREQUAL "Android")
	set(ANDROID ON)
endif ()

# Define target compiler
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CCR_COMPILER_VERSION "msvc")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CCR_COMPILER_VERSION "gcc")
else()
  message (FATAL_ERROR "Invalid compiler.")
endif ()

# Define libraries binary key
set(CCR_LIBS_BINARY_KEY ${CCR_OS_VERSION}-${CCR_COMPILER_VERSION}-${CCR_CHIPSET_VERSION})

# Check if 3rdParty libraries exist for the chosen chipset, OS, compiler and CUDA Toolkit versions
# OpenCV
set (CCR_OPENCV_PATH ${CCR_THIRD_PARTY}/opencv/${CCR_OPENCV_VERSION}/${CCR_LIBS_BINARY_KEY}-${CCR_CUDA_VERSION_ID})
set (CCR_OPENCV_PATH ${CCR_OPENCV_PATH} PARENT_SCOPE)

if (NOT EXISTS ${CCR_OPENCV_PATH})
    set (CCR_OPENCV_PATH ${CCR_THIRD_PARTY}/opencv/${CCR_OPENCV_VERSION}/${CCR_LIBS_BINARY_KEY}-nocuda)
    set (CCR_OPENCV_PATH ${CCR_OPENCV_PATH} PARENT_SCOPE)
    if (EXISTS ${CCR_OPENCV_PATH})
        message(STATUS "No CUDA compatible OpenCV library version found in the 3rdParty folder. Using the NOCUDA OpenCV build.")
    else()
        message(FATAL_ERROR "No valid OpenCV library version found in the 3rdParty folder. Exiting.")
    endif()
endif()

set(CCR_OPENCV_INCLUDE_PATH ${CCR_THIRD_PARTY}/opencv/${CCR_OPENCV_VERSION}/include)
set(CCR_OPENCV_INCLUDE_PATH ${CCR_OPENCV_INCLUDE_PATH} PARENT_SCOPE)
if(NOT EXISTS ${CCR_OPENCV_INCLUDE_PATH})
   message(FATAL_ERROR "Failed to load the requested OpenCV version: ${CCR_OPENCV_INCLUDE_PATH}")
endif()

set(CCR_OPENCV_LIBRARY_PATH ${CCR_THIRD_PARTY}/opencv/${CCR_OPENCV_VERSION}/${CCR_LIBS_BINARY_KEY}-${CCR_CUDA_VERSION_ID}/lib)
set(CCR_OPENCV_LIBRARY_PATH ${CCR_OPENCV_LIBRARY_PATH} PARENT_SCOPE)
if(NOT EXISTS ${CCR_OPENCV_LIBRARY_PATH})
   message(FATAL_ERROR "Failed to load OpenCV for the given chipset configuration: ${CCR_OPENCV_LIBRARY_PATH}")
endif()

if (NOT ANDROID)
    #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin PARENT_SCOPE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_SOURCE_DIR}> PARENT_SCOPE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} PARENT_SCOPE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib PARENT_SCOPE)
endif (NOT ANDROID)

if (ANDROID)
	set(OPENSSL_ROOT_DIR "${CMAKE_SOURCE_DIR}/3rdParty/android/${ANDROID_ABI}/openssl")
elseif (MSVC)
	set(OPENSSL_ROOT_DIR "${CMAKE_SOURCE_DIR}/3rdParty/datachannel/openssl/win-x64")
elseif (RASPBERRY_PI)
	set(OPENSSL_ROOT_DIR "${CMAKE_SOURCE_DIR}/3rdParty/datachannel/openssl/linux-arm-x86")
elseif (GENIUS_BOARD)
	set(OPENSSL_ROOT_DIR "${CMAKE_SOURCE_DIR}/3rdParty/datachannel/openssl/linux-aarch64")
elseif (UNIX)
    set(OPENSSL_ROOT_DIR "${CMAKE_SOURCE_DIR}/3rdParty/datachannel/openssl/linux-x64")
endif ()

list(APPEND CMAKE_FIND_ROOT_PATH "${OPENSSL_ROOT_DIR}")

# 3rdParty includes
set(CCR_THIRD_PARTY_INCLUDE_DIRECTORIES 
    ${CCR_THIRD_PARTY}/Eigen
    ${CCR_OPENCV_INCLUDE_PATH}
    ${CCR_THIRD_PARTY}/Eigen
    ${CCR_THIRD_PARTY}/spdlog-1.4.2/include
    ${CCR_THIRD_PARTY}/aruco-3.1.4/include
    ${CCR_THIRD_PARTY}/libcsv/include
    ${CCR_THIRD_PARTY}/openh264/include
    ${CCR_THIRD_PARTY}/octomap/include
    ${CCR_THIRD_PARTY}/libconfig/include
    ${CCR_THIRD_PARTY}/triclops/include
    ${CCR_THIRD_PARTY}/enet-1.3.14/include
    ${CCR_THIRD_PARTY}/Fast/include
    ${CCR_THIRD_PARTY}/zstd-dev/lib
    ${CCR_THIRD_PARTY}/ffmpeg/include
    ${CCR_THIRD_PARTY}/nlohmann-json/include
    ${CCR_THIRD_PARTY}/aruco-3.1.4/src
    ${CCR_THIRD_PARTY}/cvplot-0.0.3/include
    ${CCR_THIRD_PARTY}/datachannel/websocketpp
    ${CCR_THIRD_PARTY}/datachannel/msgpack/include
    ${CCR_THIRD_PARTY}/datachannel/libdatachannel/include
    ${CCR_THIRD_PARTY}/datachannel/libdatachannel/deps/usrsctp/usrsctplib
    ${CCR_THIRD_PARTY}/libcurl/include
    ${CCR_THIRD_PARTY}/libzip/include
    ${CCR_THIRD_PARTY}/unitree/include
    ${CCR_THIRD_PARTY}/realsense-2.50.0/include
    ${CCR_THIRD_PARTY}/datachannel/asio/include
    ${CCR_THIRD_PARTY}/boost-1.78.0/include
    ${CCR_THIRD_PARTY}/gtsam/include
    ${CCR_THIRD_PARTY}/Sophus/include
    ${CCR_THIRD_PARTY}/DBoW2/include
    ${CCR_THIRD_PARTY}/g2o/include
    if (WIN32)
        ${CCR_THIRD_PARTY}/pthreads/include
        ${CCR_THIRD_PARTY}/qt-5.12.2/win-msvc141-x64/include
    endif (WIN32)
    ${CCR_THIRD_PARTY}/libceres/${CCR_CERES_VERSION}/include)
set(CCR_THIRD_PARTY_INCLUDE_DIRECTORIES ${CCR_THIRD_PARTY_INCLUDE_DIRECTORIES} PARENT_SCOPE)

# 3rdParty library includes
set(CCR_THIRD_PARTY_LIBRARY_DIRECTORIES 
    ${CMAKE_SOURCE_DIR}/inference/lib/${CCR_LIBS_BINARY_KEY} # CyberCortex.AI library includes
    ${CCR_THIRD_PARTY}/enet-1.3.14/${CCR_LIBS_BINARY_KEY}/lib
    ${CCR_THIRD_PARTY}/libconfig/${CCR_LIBS_BINARY_KEY}/lib
    ${CCR_THIRD_PARTY}/opencv/${CCR_OPENCV_VERSION}/${CCR_LIBS_BINARY_KEY}-${CCR_CUDA_VERSION_ID}/lib
    ${CCR_THIRD_PARTY}/libtorch/${CCR_TORCH_VERSION}/${CCR_LIBS_BINARY_KEY}-${CCR_CUDA_VERSION_ID}/lib
    ${CCR_THIRD_PARTY}/onnxruntime/${CCR_ONNXRUNTIME_VERSION}/${CCR_LIBS_BINARY_KEY}-${CCR_CUDA_VERSION_ID}/lib
    ${CCR_THIRD_PARTY}/ffmpeg/${CCR_LIBS_BINARY_KEY}/lib
    ${CCR_THIRD_PARTY}/glut-2.0.0.29/${CCR_LIBS_BINARY_KEY}/lib
    ${CCR_THIRD_PARTY}/boost-1.78.0/${CCR_LIBS_BINARY_KEY}/lib
    ${CCR_THIRD_PARTY}/gtsam/${CCR_LIBS_BINARY_KEY}/lib
    ${CCR_THIRD_PARTY}/DBoW2/${CCR_LIBS_BINARY_KEY}/lib
    ${CCR_THIRD_PARTY}/g2o/${CCR_LIBS_BINARY_KEY}/lib
    ${CCR_THIRD_PARTY}/realsense-2.50.0/${CCR_LIBS_BINARY_KEY}/lib
    ${CCR_THIRD_PARTY}/torchvision/${CCR_LIBS_BINARY_KEY}-${CCR_CUDA_VERSION_ID}/lib
    if (ANDROID)
        # armeabi-v7a, arm64-v8a, x86, x86_64
        ${CCR_THIRD_PARTY}/android/${ANDROID_ABI}
    elseif (MSVC)
        ${CCR_THIRD_PARTY}/openh264/win-msvc141-x64/lib
        ${CCR_THIRD_PARTY}/octomap/${CCR_LIBS_BINARY_KEY}/lib
        ${CCR_THIRD_PARTY}/aruco-3.1.4/win-msvc141-x64/lib
        ${CCR_THIRD_PARTY}/spinnaker/lib64
        ${CCR_THIRD_PARTY}/libcurl/win-msvc141-x64/lib
        ${CCR_THIRD_PARTY}/libzip/win-msvc141-x64/lib
    elseif (RASPBERRY_PI)
        ${CCR_THIRD_PARTY}/cvplot-0.0.3/linux-gcc-arm-x86/lib
        ${CCR_THIRD_PARTY}/octomap/linux-gcc-x64/lib
        ${CCR_THIRD_PARTY}/src/ccr_filters/Filters_HardwareComm/Filter_HW_MonoCamera/src/interfaces/raspi/lib
    elseif (GENIUS_BOARD)
        ${CCR_THIRD_PARTY}/cvplot-0.0.3/linux-gcc-arm-x64/lib
        ${CCR_THIRD_PARTY}/openh264/linux-gcc-arm-x64/lib
        ${CCR_THIRD_PARTY}/unitree/linux-gcc-arm-x64/lib
        ${CCR_THIRD_PARTY}/realsense-2.50.0/linux-gcc-arm-x64/lib
    elseif (UNIX)
        ${CCR_THIRD_PARTY}/octomap/linux-gcc-x64/lib
        ${CCR_THIRD_PARTY}/openh264/linux-gcc-x64/lib
        ${CCR_THIRD_PARTY}/unitree/linux-gcc-x64/lib
    endif ()
    ${CCR_THIRD_PARTY}/libceres/${CCR_CERES_VERSION}/${CCR_LIBS_BINARY_KEY}/lib)
set(CCR_THIRD_PARTY_LIBRARY_DIRECTORIES ${CCR_THIRD_PARTY_LIBRARY_DIRECTORIES} PARENT_SCOPE)

# Build CyberCortex.AI filters
macro(add_ccr_filter name filter_path)
	set(Build_${name} FALSE CACHE BOOL "Build_${name}")
	if (Build_${name})
		add_subdirectory(${filter_path})
		get_directory_property(FN DIRECTORY ${filter_path} DEFINITION FILTER_NAME)
		#target_include_directories(${FN} PUBLIC ${CMAKE_SOURCE_DIR}/${filter_path}/include)
		target_include_directories(${FN} PUBLIC ${filter_path}/include)
		#target_compile_definitions(CcrCore PUBLIC ${name})
        #target_link_libraries(${FN} PUBLIC ${CCR_OPENCV_WORLD})
		message(STATUS "Filter '${FN}' enabled.")
	endif (Build_${name})
endmacro()
