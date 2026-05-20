# Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

### Unit Testing ###

# Compute inspector
add_executable(tu_ComputeInspector src/tu_ComputeInspector.cpp)

if (MSVC)
    target_link_libraries(tu_ComputeInspector
        PUBLIC
            CycToolkit
            pdh)
    target_compile_definitions(tu_ComputeInspector PRIVATE
        _SILENCE_STDEXT_ARR_ITERS_DEPRECATION_WARNING)
endif ()
