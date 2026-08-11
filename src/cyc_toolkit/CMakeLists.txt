# Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

# CUDA dependency
find_package(CUDAToolkit)

# Unit Testing
if (Build_UnitTests)
    add_subdirectory(src/tu)
endif (Build_UnitTests)

# QtPlot
add_subdirectory(src/os/qtplot EXCLUDE_FROM_ALL)

if (MSVC)
    set(LIB_TYPE STATIC)
else ()
    set(LIB_TYPE SHARED)
endif ()

add_library(CycToolkit ${LIB_TYPE}
    src/os/CSingletonBase.h
    src/os/CSingletonRegistry.h
    src/os/CConversions.h
    src/os/CConversions.cpp
    src/os/CCsvReader.h
    src/os/CCsvReader.cpp
    src/os/CCsvWriter.h
    src/os/CCsvWriter.cpp
    src/os/CFileUtils.h
    src/os/CFileUtils.cpp
    src/os/CStringUtils.h
    src/os/CStringUtils.cpp
    src/os/CTimer.h
    src/os/CTimer.cpp
    src/os/CMacUtils.h
    src/os/CMacUtils.cpp
    src/os/CDataBlockReader.h
    src/os/CDataBlockReader.cpp
    src/os/CComputeInspect.h
    src/os/CComputeInspect.cpp
    src/math/CGeometry.h
    src/math/CGeometry.cpp
    src/math/CPolynomialFitting.h
    src/math/CPolynomialFitting.cpp
    src/math/CRunningStat.h
    src/math/CRandom.h
    src/math/CRandom.cpp
    src/math/CMathUtils.h
    src/math/CMathUtils.cpp
    src/sensors/CBaseSensorModel.h
    src/sensors/CBaseSensorModel.cpp
    src/sensors/CPinholeCameraSensorModel.h
    src/sensors/CPinholeCameraSensorModel.cpp
    src/sensors/CLidarSensorModel.h
    src/sensors/CLidarSensorModel.cpp
    src/sensors/CImuSensorModel.h
    src/sensors/CImuSensorModel.cpp
    src/sensors/CBarometerSensorModel.h
    src/sensors/CBarometerSensorModel.cpp
    src/vision/CImageProcessing.h
    src/vision/CImageProcessing.cpp
    src/vision/CDepthImageProcessing.h
    src/vision/CDepthImageProcessing.cpp
    src/vision/CProjectiveGeometry.h
    src/vision/CProjectiveGeometry.cpp
    src/vision/CTriangulation.h
    src/vision/CTriangulation.cpp
    src/vision/CImageDisplayUtils.h
    src/vision/CImageDisplayUtils.cpp
    src/vision/clustering/kdtree.h
    src/vision/clustering/kdtree.cpp
    src/vision/clustering/dbscan.hpp
    src/control/CBaseStateSpaceModel.h
    src/control/CBaseStateSpaceModel.cpp
    src/control/CBaseController.h
    src/control/CBaseController.cpp
    src/control/CStateSpaceController.h
    src/control/CStateSpaceController.cpp
    src/control/CRobotConfig.h
    src/control/CRobotConfig.cpp
    src/control/CStateSpaceModel.h
    src/control/CStateSpaceModelArm.h
    src/control/CSimulation.h
    src/control/CSimulationArm.h
    src/control/CKalmanOps.h
    src/control/CKalmanOps.cpp
    src/control/CLinearFirstOrderSystem.h
    src/control/CLinearFirstOrderSystem.cpp
    src/control/CDenavitHartenberg.h
    src/control/CDenavitHartenberg.cpp
    src/env/CGridMapUtils.h
    src/env/CGridMapUtils.cpp
    src/env/CCycOcTree.h
    src/env/CCycOcTree.cpp
    src/env/COcTreeUtils.h
    src/env/COcTreeUtils.cpp
    src/env/CObjectClasses.h
    src/env/CObjectClasses.cpp
    src/env/CObjects.h
    src/env/CObjects.cpp
    src/env/CGeolocation.h
    src/env/CGeolocation.cpp
    src/env/CPose.h
    src/env/CPose.cpp
    src/env/CQuaternion.h
    src/env/CQuaternion.cpp
    src/plan/CPlanningUtils.h
    src/plan/CPlanningUtils.cpp
    src/plan/CWaypointsPlanner.h
    src/plan/CWaypointsPlanner.cpp
    src/dl/CDnnConfig.h
    src/dl/CDnnConfig.cpp)

target_include_directories(CycToolkit
    PUBLIC
        ${CyC_OPENCV_INCLUDE_PATH}
        ${CUDAToolkit_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/src/cyc_core/src
        ${CMAKE_SOURCE_DIR}/src/cyc_toolkit/src
        ${CyC_THIRD_PARTY_INCLUDE_DIRECTORIES})
        
target_link_directories(CycToolkit
    PUBLIC
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES}
        ${CUDAToolkit_LIBRARY_DIR})

target_link_libraries(CycToolkit
        PUBLIC
            ${CyC_OPENCV_WORLD}
            ${CyC_LIBCONFIG}
            octomap
            octomath)

if (MSVC)
    target_link_libraries(CycToolkit PRIVATE pdh)
    # Silence spdlog-1.4.2 / bundled fmt using deprecated stdext::checked_array_iterator (STL4043)
    target_compile_definitions(CycToolkit PRIVATE
        _SILENCE_STDEXT_ARR_ITERS_DEPRECATION_WARNING
        _CRT_SECURE_NO_WARNINGS   # fopen, ctime, sprintf used intentionally
    )
endif ()

if(CUDA_FOUND)
    target_link_libraries(CycToolkit PRIVATE nvml)
endif(CUDA_FOUND)
