cmake_minimum_required(VERSION 3.16)
project(xrealair C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_OSX_ARCHITECTURES "x86_64;arm64")

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(modules/Fusion/Fusion)

add_subdirectory(modules/hidapi)
set_target_properties(hidapi_darwin PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    STATIC_LIBRARY TRUE
)

set(BUILD_SHARED_LIBS OFF)
set(DISABLE_SHARED ON)
set(BUILD_STATIC_LIBS ON)
add_subdirectory(modules/json-c)
set_target_properties(json-c-static PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    STATIC_LIBRARY TRUE
)

add_library(xrealair SHARED
    src/crc32.c
    src/device.c
    src/device_imu.c
    src/device_mcu.c
    src/hid_ids.c
)
set_target_properties(xrealair PROPERTIES
	OUTPUT_NAME "xrealair"
    VERSION 0.1.1
    SOVERSION 0.1.1
)

target_compile_options(xrealair PRIVATE -Wall -Wextra -fPIC -DNDEBUG)

find_library(IOKIT IOKit)
find_library(COREFOUNDATION CoreFoundation)

target_include_directories(xrealair
    BEFORE PUBLIC include
    SYSTEM BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/hidapi
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/Fusion
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/json-c
)

target_link_libraries(xrealair PRIVATE
    -static hidapi_darwin
    -static Fusion
    -static json-c-static
    m ${IOKIT} ${COREFOUNDATION}
)

set_target_properties(xrealair PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "@loader_path"
    LINK_FLAGS "-Wl,-dead_strip"
)

set(XREAL_AIR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(XREAL_AIR_LIBRARY xrealair)

add_subdirectory(examples)