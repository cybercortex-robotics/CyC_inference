# Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
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
### OpenCV ###
set (CyC_OPENCV_PATH ${CyC_THIRD_PARTY}/opencv/${CyC_OPENCV_VERSION}/${CyC_LIBS_BINARY_KEY}-${CyC_CUDA_VERSION_ID})
set (CyC_OPENCV_PATH ${CyC_OPENCV_PATH} PARENT_SCOPE)

if (NOT EXISTS ${CyC_OPENCV_PATH})
    set (CyC_OPENCV_PATH ${CyC_THIRD_PARTY}/opencv/${CyC_OPENCV_VERSION}/${CyC_LIBS_BINARY_KEY}-nocuda)
    set (CyC_OPENCV_PATH ${CyC_OPENCV_PATH} PARENT_SCOPE)
    if (EXISTS ${CyC_OPENCV_PATH})
        message(STATUS "No CUDA compatible OpenCV library version found in the 3rdParty folder. Using the NOCUDA OpenCV build.")
    else()
        message(STATUS "No valid OpenCV library version found in the 3rdParty folder. Searching for a system level installation.")
        find_package(OpenCV REQUIRED)
        set(CyC_OPENCV_SYS_LEVEL true)
    endif()
endif()

if (${CyC_OPENCV_SYS_LEVEL})
    set(CyC_OPENCV_INCLUDE_PATH ${OpenCV_INCLUDE_DIRS})
else()
    set(CyC_OPENCV_INCLUDE_PATH ${CyC_THIRD_PARTY}/opencv/${CyC_OPENCV_VERSION}/include)
endif()
set(CyC_OPENCV_INCLUDE_PATH ${CyC_OPENCV_INCLUDE_PATH} PARENT_SCOPE)
if(NOT EXISTS ${CyC_OPENCV_INCLUDE_PATH})
   message(FATAL_ERROR "Failed to load the requested OpenCV version: ${CyC_OPENCV_INCLUDE_PATH}")
endif()

if (${CyC_OPENCV_SYS_LEVEL})
    set(CyC_OPENCV_LIBRARY_PATH ${OpenCV_LIBRARY_DIRS})
    if(TARGET opencv_world)
        set(CyC_OPENCV_WORLD opencv_world)
    else()
        set(CyC_OPENCV_WORLD ${OpenCV_LIBS})
    endif()
    set(CyC_OPENCV_WORLD ${CyC_OPENCV_WORLD} PARENT_SCOPE)
else()
    set(CyC_OPENCV_LIBRARY_PATH ${CyC_THIRD_PARTY}/opencv/${CyC_OPENCV_VERSION}/${CyC_LIBS_BINARY_KEY}-${CyC_CUDA_VERSION_ID}/lib)

    if(NOT EXISTS ${CyC_OPENCV_LIBRARY_PATH})
        message(FATAL_ERROR "Failed to load OpenCV for the given chipset configuration: ${CyC_OPENCV_LIBRARY_PATH}")
    endif()
endif()
set(CyC_OPENCV_LIBRARY_PATH ${CyC_OPENCV_LIBRARY_PATH} PARENT_SCOPE)

### Eigen ###
set (CyC_Eigen_PATH ${CyC_THIRD_PARTY}/Eigen)
if (NOT EXISTS ${CyC_Eigen_PATH})
    message(STATUS "No valid Eigen3 library version found in the 3rdParty folder. Searching for a system level installation.")
    find_package(Eigen3 REQUIRED)
    set (CyC_Eigen_PATH ${EIGEN3_INCLUDE_DIR})
endif()
set (CyC_Eigen_PATH ${CyC_Eigen_PATH} PARENT_SCOPE)

### Spdlog ###
set (CyC_Spdlog_PATH ${CyC_THIRD_PARTY}/spdlog-1.4.2/include)
if (NOT EXISTS ${CyC_Spdlog_PATH})
    message(STATUS "No valid Spdlog library version found in the 3rdParty folder. Searching for a system level installation.")
    find_package(spdlog REQUIRED)
endif()
set (CyC_Spdlog_PATH ${CyC_Spdlog_PATH} PARENT_SCOPE)

### LibConfig++ ###
set (CyC_LibConfig_PATH ${CyC_THIRD_PARTY}/libconfig/include)
if (NOT EXISTS ${CyC_LibConfig_PATH})
    message(STATUS "No valid LibConfig library version found in the 3rdParty folder. Searching for a system level installation.")
    find_package(PkgConfig REQUIRED)
    set (CyC_LibConfig_PATH ${LIBCONFIG_INCLUDE_DIRS})
endif()
set (CyC_LibConfig_PATH ${CyC_LibConfig_PATH} PARENT_SCOPE)

### msgpack ###
set (CyC_MsgPack_PATH ${CyC_THIRD_PARTY}/msgpack/include)
if (NOT EXISTS ${CyC_MsgPack_PATH})
    message(STATUS "No valid MsgPack library version found in the 3rdParty folder. Searching for a system level installation.")
    find_package(msgpack QUIET)
    if(NOT msgpack_FOUND)
        find_package(msgpack-cxx REQUIRED)
        add_library(msgpack ALIAS msgpack-cxx) 
        set(msgpack_FOUND TRUE)
    endif()
endif()
set (CyC_MsgPack_PATH ${CyC_MsgPack_PATH} PARENT_SCOPE)

### OctoMap ###
set (CyC_OctoMap_PATH ${CyC_THIRD_PARTY}/octomap/include)
if (NOT EXISTS ${CyC_OctoMap_PATH})
    message(STATUS "No valid OctoMap library version found in the 3rdParty folder. Searching for a system level installation.")
    find_package(octomap REQUIRED)
endif()
set (CyC_OctoMap_PATH ${CyC_OctoMap_PATH} PARENT_SCOPE)



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
    ${CyC_Eigen_PATH}
    ${CyC_Spdlog_PATH}
    ${CyC_LibConfig_PATH}
    ${CyC_MsgPack_PATH}
    ${CyC_THIRD_PARTY}/openh264/include
    ${CyC_THIRD_PARTY}/octomap/include
    ${CyC_THIRD_PARTY}/ffmpeg/include
    ${CyC_THIRD_PARTY}/nlohmann-json/include
    ${CyC_THIRD_PARTY}/cvplot-0.0.3/include
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
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CyC_LIBS_BINARY_KEY} # CyberCortex.AI library includes
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
    ${CyC_THIRD_PARTY}/cvplot-0.0.3/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/pthreads/${CyC_LIBS_BINARY_KEY}/lib
    ${CyC_THIRD_PARTY}/msgpack/${CyC_LIBS_BINARY_KEY}/lib)

if (ANDROID)
    set(CyC_THIRD_PARTY_LIBRARY_DIRECTORIES ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES}
        # armeabi-v7a, arm64-v8a, x86, x86_64
        ${CyC_THIRD_PARTY}/android/${ANDROID_ABI})
endif (ANDROID)

set(CyC_THIRD_PARTY_LIBRARY_DIRECTORIES ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES} PARENT_SCOPE)

# Build CyberCortex.AI filters
if (NOT TARGET BUILD_ALL_FILTERS)
    add_custom_target(BUILD_ALL_FILTERS)
endif ()
macro(add_cyc_filter name filter_path)
	set(Build_${name} FALSE CACHE BOOL "Build_${name}")
	if (Build_${name})
		add_subdirectory(${filter_path})
		get_directory_property(FN DIRECTORY ${filter_path} DEFINITION FILTER_NAME)
		#target_include_directories(${FN} PUBLIC ${CMAKE_SOURCE_DIR}/${filter_path}/include)
		#target_include_directories(${FN} PUBLIC ${filter_path}/include)
		#target_compile_definitions(CycCore PUBLIC ${name})
        #target_link_libraries(${FN} PUBLIC ${CyC_OPENCV_WORLD})
        if (TARGET BUILD_ALL_FILTERS)
            add_dependencies(BUILD_ALL_FILTERS ${FN})
        endif ()
		message(STATUS "Filter '${FN}' enabled.")
	endif (Build_${name})
endmacro()

function(download_cyc_filter ZIP_URL FILTERS_PATH)
    string(REGEX REPLACE ".*/" "" _zip_name "${ZIP_URL}")
    string(REGEX REPLACE "\\.zip$" "" _filter_name "${_zip_name}")
    set(_zip_path "${CMAKE_SOURCE_DIR}/bin/${_zip_name}")
    set(_extract_dir "${CMAKE_SOURCE_DIR}/bin/${FILTERS_PATH}")
    set(_need_extract FALSE)

    if (NOT EXISTS "${_zip_path}")
        message(STATUS "Downloading ${_zip_name} ...")
        file(DOWNLOAD "${ZIP_URL}" "${_zip_path}"
            SHOW_PROGRESS
            STATUS _download_status)
        list(GET _download_status 0 _download_error)
        if (NOT _download_error EQUAL 0)
            list(GET _download_status 1 _download_msg)
            message(FATAL_ERROR "Failed to download ${_zip_name}: ${_download_msg}")
        endif()
        set(_need_extract TRUE)
    else()
        file(TIMESTAMP "${_zip_path}" _local_ts "%a, %d %b %Y %H:%M:%S GMT" UTC)
        set(_zip_new "${CMAKE_BINARY_DIR}/${_filter_name}_new.zip")
        message(STATUS "Checking for ${_zip_name} updates (local: ${_local_ts}) ...")
        file(DOWNLOAD "${ZIP_URL}" "${_zip_new}"
            SHOW_PROGRESS
            HTTPHEADER "If-Modified-Since: ${_local_ts}"
            STATUS _download_status)
        list(GET _download_status 0 _download_error)
        if (NOT _download_error EQUAL 0)
            list(GET _download_status 1 _download_msg)
            message(WARNING "Could not check for ${_zip_name} updates: ${_download_msg}")
            file(REMOVE "${_zip_new}")
        else()
            # A 200 response contains a valid zip; a 304 produces no zip content.
            # Check for the zip magic bytes PK (0x504b0304) to tell them apart.
            file(READ "${_zip_new}" _zip_magic LIMIT 4 HEX)
            if (_zip_magic STREQUAL "504b0304")
                message(STATUS "Newer ${_zip_name} found — replacing local copy.")
                file(RENAME "${_zip_new}" "${_zip_path}")
                set(_need_extract TRUE)
            else()
                message(STATUS "${_zip_name} is up to date, skipping download.")
                file(REMOVE "${_zip_new}")
            endif()
        endif()
    endif()

    if (_need_extract)
        message(STATUS "Extracting ${_zip_name} to ${_extract_dir} ...")
        set(_temp_dir "${CMAKE_BINARY_DIR}/${_filter_name}_temp")
        file(MAKE_DIRECTORY "${_temp_dir}")
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E tar xf "${_zip_path}"
            WORKING_DIRECTORY "${_temp_dir}"
            RESULT_VARIABLE _extract_result)
        if (NOT _extract_result EQUAL 0)
            message(FATAL_ERROR "Failed to extract ${_zip_name}")
        endif()
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${_temp_dir}/${_filter_name}" "${_extract_dir}")
        file(REMOVE_RECURSE "${_temp_dir}")
    endif()
endfunction()

if (NOT ANDROID)
    #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_SOURCE_DIR}/bin>)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
endif (NOT ANDROID)

# CycCore and CycToolkit libraries
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/cyc_toolkit/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/cyc_core/src)
add_subdirectory(src/cyc_core)
add_subdirectory(src/cyc_toolkit)

# CyberCortex Apps
if (NOT ANDROID)
    add_subdirectory(src/cyc_apps)
endif (NOT ANDROID)
