# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(CMAKE_POSITION_INDEPENDENT_CODE ON PARENT_SCOPE)

# Check if the 3rdParty folder is set
if (NOT EXISTS ${CyC_THIRD_PARTY})
    set(CyC_THIRD_PARTY ${CMAKE_SOURCE_DIR}/3rdParty)
    set(CyC_THIRD_PARTY ${CyC_THIRD_PARTY} PARENT_SCOPE)
endif ()

set(CyC_OPENCV_WORLD "opencv_world")
set(CyC_OPENCV_WORLD ${CyC_OPENCV_WORLD} PARENT_SCOPE)

set(CyC_LIBCONFIG "config++")
set(CyC_LIBCONFIG ${CyC_LIBCONFIG} PARENT_SCOPE)

# Check if CUDA version is given
if (CyC_CUDA_VERSION STREQUAL "")
  set(CyC_CUDA_VERSION_ID "nocuda")
else()
  string(REPLACE "." "" CyC_CUDA_VERSION_ID ${CyC_CUDA_VERSION})
  set(CyC_CUDA_VERSION_ID "cuda${CyC_CUDA_VERSION_ID}")
endif()
set(CyC_CUDA_VERSION_ID ${CyC_CUDA_VERSION_ID} PARENT_SCOPE)

string(REPLACE "." "" CyC_OPENCV_VERSION_ID ${CyC_OPENCV_VERSION})

set(Build_UnitTests FALSE CACHE BOOL "Build_UnitTests")
set(Build_Examples FALSE CACHE BOOL "Build_Examples")

# Define target chipset
if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CyC_CHIPSET_VERSION "x64")
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "AMD64")
    set(CyC_CHIPSET_VERSION "x64")
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "IA64")
    set(CyC_CHIPSET_VERSION "x64")
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")  # ARM x64 chipset (NVIDIA Jetson AGX)
	set(CyC_CHIPSET_VERSION "arm-x64")
    set(GENIUS_BOARD TRUE)
	add_definitions(-DGENIUS_BOARD)
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7l")   # ARM x86 chipset (Raspberry PI)
	set(CyC_CHIPSET_VERSION "arm-x86")
    set(RASPBERRY_PI TRUE)
	add_definitions(-DRASPBERRY_PI)
endif ()
set(CyC_CHIPSET_VERSION ${CyC_CHIPSET_VERSION} PARENT_SCOPE)

# Define target operating system
if (WIN32)
    set(CyC_OS_VERSION "win")
    set(CyC_OPENCV_WORLD "opencv_world${CyC_OPENCV_VERSION_ID}")
    set(CyC_OPENCV_WORLD ${CyC_OPENCV_WORLD} PARENT_SCOPE)
    set(CyC_LIBCONFIG "libconfig++")
    set(CyC_LIBCONFIG ${CyC_LIBCONFIG} PARENT_SCOPE)
elseif (UNIX)
    set(CyC_OS_VERSION "linux")
elseif (ANDROID)
    set(CyC_OS_VERSION "android")
endif ()
set(CyC_OS_VERSION ${CyC_OS_VERSION} PARENT_SCOPE)

if (CMAKE_SYSTEM_NAME STREQUAL "Android")
	set(ANDROID ON)
endif ()

# Define target compiler
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CyC_COMPILER_VERSION "msvc")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CyC_COMPILER_VERSION "gcc")
else()
  message (FATAL_ERROR "Invalid compiler.")
endif ()

# Define libraries binary key
set(CyC_LIBS_BINARY_KEY ${CyC_OS_VERSION}-${CyC_COMPILER_VERSION}-${CyC_CHIPSET_VERSION})
set(CyC_LIBS_BINARY_KEY ${CyC_LIBS_BINARY_KEY} PARENT_SCOPE)

# Check if 3rdParty libraries exist for the chosen chipset, OS, compiler and CUDA Toolkit versions
# OpenCV
set (CyC_OPENCV_PATH ${CyC_THIRD_PARTY}/opencv/${CyC_OPENCV_VERSION}/${CyC_LIBS_BINARY_KEY}-${CyC_CUDA_VERSION_ID})
set (CyC_OPENCV_PATH ${CyC_OPENCV_PATH} PARENT_SCOPE)

if (NOT EXISTS ${CyC_OPENCV_PATH})
    set (CyC_OPENCV_PATH ${CyC_THIRD_PARTY}/opencv/${CyC_OPENCV_VERSION}/${CyC_LIBS_BINARY_KEY}-nocuda)
    set (CyC_OPENCV_PATH ${CyC_OPENCV_PATH} PARENT_SCOPE)
    if (EXISTS ${CyC_OPENCV_PATH})
        message(STATUS "No CUDA compatible OpenCV library version found in the 3rdParty folder. Using the NOCUDA OpenCV build.")
    else()
        message(FATAL_ERROR "No valid OpenCV library version found in the 3rdParty folder. Exiting.")
    endif()
endif()

set(CyC_OPENCV_INCLUDE_PATH ${CyC_THIRD_PARTY}/opencv/${CyC_OPENCV_VERSION}/include)
set(CyC_OPENCV_INCLUDE_PATH ${CyC_OPENCV_INCLUDE_PATH} PARENT_SCOPE)
if(NOT EXISTS ${CyC_OPENCV_INCLUDE_PATH})
   message(FATAL_ERROR "Failed to load the requested OpenCV version: ${CyC_OPENCV_INCLUDE_PATH}")
endif()

set(CyC_OPENCV_LIBRARY_PATH ${CyC_THIRD_PARTY}/opencv/${CyC_OPENCV_VERSION}/${CyC_LIBS_BINARY_KEY}-${CyC_CUDA_VERSION_ID}/lib)
set(CyC_OPENCV_LIBRARY_PATH ${CyC_OPENCV_LIBRARY_PATH} PARENT_SCOPE)
if(NOT EXISTS ${CyC_OPENCV_LIBRARY_PATH})
   message(FATAL_ERROR "Failed to load OpenCV for the given chipset configuration: ${CyC_OPENCV_LIBRARY_PATH}")
endif()

if (NOT ANDROID)
    #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin PARENT_SCOPE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_SOURCE_DIR}> PARENT_SCOPE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} PARENT_SCOPE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib PARENT_SCOPE)
endif (NOT ANDROID)

# Set OpenSSL root directory
if (ANDROID)
	set(OPENSSL_ROOT_DIR "${CyC_THIRD_PARTY}/android/${ANDROID_ABI}/openssl" PARENT_SCOPE)
elseif (MSVC)
	set(OPENSSL_ROOT_DIR "${CyC_THIRD_PARTY}/openssl/win-x64" PARENT_SCOPE)
elseif (RASPBERRY_PI)
	set(OPENSSL_ROOT_DIR "${CyC_THIRD_PARTY}/openssl/linux-arm-x86" PARENT_SCOPE)
elseif (GENIUS_BOARD)
	set(OPENSSL_ROOT_DIR "${CyC_THIRD_PARTY}/openssl/linux-aarch64" PARENT_SCOPE)
elseif (UNIX)
    set(OPENSSL_ROOT_DIR "${CyC_THIRD_PARTY}/openssl/linux-x64" PARENT_SCOPE)
endif ()
list(APPEND CMAKE_FIND_ROOT_PATH "${OPENSSL_ROOT_DIR}" PARENT_SCOPE)

# 3rdParty includes
set(CyC_THIRD_PARTY_INCLUDE_DIRECTORIES 
    ${CyC_OPENCV_INCLUDE_PATH}
    ${CyC_THIRD_PARTY}/Eigen
    ${CyC_THIRD_PARTY}/spdlog-1.4.2/include
    ${CyC_THIRD_PARTY}/aruco-3.1.4/include
    ${CyC_THIRD_PARTY}/libcsv/include
    ${CyC_THIRD_PARTY}/openh264/include
    ${CyC_THIRD_PARTY}/octomap/include
    ${CyC_THIRD_PARTY}/libconfig/include
    ${CyC_THIRD_PARTY}/enet-1.3.14/include
    ${CyC_THIRD_PARTY}/zstd-dev/lib
    ${CyC_THIRD_PARTY}/ffmpeg/include
    ${CyC_THIRD_PARTY}/nlohmann-json/include
    ${CyC_THIRD_PARTY}/aruco-3.1.4/src
    ${CyC_THIRD_PARTY}/cvplot-0.0.3/include
    ${CyC_THIRD_PARTY}/msgpack/include
    ${CyC_THIRD_PARTY}/libcurl/include
    ${CyC_THIRD_PARTY}/libzip/include
    ${CyC_THIRD_PARTY}/boost-1.78.0/include
    ${CyC_THIRD_PARTY}/glut-2.0.0.29/include)
if (WIN32)
    set(CyC_THIRD_PARTY_INCLUDE_DIRECTORIES ${CyC_THIRD_PARTY_INCLUDE_DIRECTORIES}
        ${CyC_THIRD_PARTY}/pthreads/include
        ${CyC_THIRD_PARTY}/qt-5.12.2/win-msvc141-x64/include)
endif (WIN32)
set(CyC_THIRD_PARTY_INCLUDE_DIRECTORIES ${CyC_THIRD_PARTY_INCLUDE_DIRECTORIES} PARENT_SCOPE)

# 3rdParty library includes
set(CyC_THIRD_PARTY_LIBRARY_DIRECTORIES 
    ${CMAKE_SOURCE_DIR}/CyC_inference/lib/${CyC_LIBS_BINARY_KEY} # CyberCortex.AI library includes
    ${CyC_THIRD_PARTY}/enet-1.3.14/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/libconfig/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/opencv/${CyC_OPENCV_VERSION}/${CyC_LIBS_BINARY_KEY}-${CyC_CUDA_VERSION_ID}/lib
    ${CyC_THIRD_PARTY}/libtorch/${CyC_TORCH_VERSION}/${CyC_LIBS_BINARY_KEY}-${CyC_CUDA_VERSION_ID}/lib
    ${CyC_THIRD_PARTY}/onnxruntime/${CyC_ONNXRUNTIME_VERSION}/${CyC_LIBS_BINARY_KEY}-${CyC_CUDA_VERSION_ID}/lib
    ${CyC_THIRD_PARTY}/ffmpeg/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/glut-2.0.0.29/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/boost-1.78.0/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/torchvision/${CyC_LIBS_BINARY_KEY}-${CyC_CUDA_VERSION_ID}/lib
    ${CyC_THIRD_PARTY}/octomap/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/openh264/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/libceres/${CyC_CERES_VERSION}/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/cvplot-0.0.3/${CyC_LIBS_BINARY_KEY}/lib)

if (WIN32)
    set(CyC_THIRD_PARTY_LIBRARY_DIRECTORIES ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES}
        ${CyC_THIRD_PARTY}/aruco-3.1.4/win-msvc141-x64/lib
        ${CyC_THIRD_PARTY}/spinnaker/lib64
        ${CyC_THIRD_PARTY}/libcurl/win-msvc141-x64/lib
        ${CyC_THIRD_PARTY}/libzip/win-msvc141-x64/lib)
endif (WIN32)

if (ANDROID)
    set(CyC_THIRD_PARTY_LIBRARY_DIRECTORIES ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES}
        # armeabi-v7a, arm64-v8a, x86, x86_64
        ${CyC_THIRD_PARTY}/android/${ANDROID_ABI})
endif (ANDROID)

set(CyC_THIRD_PARTY_LIBRARY_DIRECTORIES ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES} PARENT_SCOPE)


# Build CyberCortex.AI filters
macro(add_cyc_filter name filter_path)
	set(Build_${name} FALSE CACHE BOOL "Build_${name}")
	if (Build_${name})
		add_subdirectory(${filter_path})
		get_directory_property(FN DIRECTORY ${filter_path} DEFINITION FILTER_NAME)
		#target_include_directories(${FN} PUBLIC ${CMAKE_SOURCE_DIR}/${filter_path}/include)
		#target_include_directories(${FN} PUBLIC ${filter_path}/include)
		#target_compile_definitions(CycCore PUBLIC ${name})
        target_link_directories(${FN} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CyC_LIBS_BINARY_KEY})
        #target_link_libraries(${FN} PUBLIC ${CyC_OPENCV_WORLD})
        #add_dependencies(BUILD_ALL_FILTERS ${FN})
		message(STATUS "Filter '${FN}' enabled.")
	endif (Build_${name})
endmacro()
