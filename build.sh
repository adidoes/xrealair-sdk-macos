#!/bin/bash

# Create temporary CMakeLists.txt
cat > modules/magnetometer-calibrate/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.16)
project(magnetometerCalibration C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

add_library(
    magnetometerCalibration
    src/magnet.c
)

target_include_directories(magnetometerCalibration
    SYSTEM BEFORE PRIVATE
    ${LIBGSL_INCLUDE_DIRS}
)

target_link_libraries(magnetometerCalibration
    PRIVATE ${LIBGSL_LIBRARIES}
)

set(MAG_CAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
set(MAG_CAL_LIBRARY magnetometerCalibration PARENT_SCOPE)
EOF

# Main build
rm -rf build && \
mkdir build && \
cd build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -Wno-dev && \
make -j$(sysctl -n hw.physicalcpu)

# Restore original CMakeLists.txt using git
cd ..
git -C modules/magnetometer-calibrate checkout CMakeLists.txt