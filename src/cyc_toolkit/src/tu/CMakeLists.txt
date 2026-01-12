# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

### Unit Testing ###

# Compute inspector
add_executable(tu_ComputeInspector
		src/tu_ComputeInspector.cpp)

if (MSVC)
    target_link_libraries(tu_ComputeInspector
        PUBLIC
            CycToolkit
            pdh)
endif ()
