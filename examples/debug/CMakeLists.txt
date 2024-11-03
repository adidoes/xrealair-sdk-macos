cmake_minimum_required(VERSION 3.16)
project(xrealAirDebug C)

set(CMAKE_C_STANDARD 17)

add_executable(
	xrealAirDebug
		src/debug.c
)

target_include_directories(xrealAirDebug
		BEFORE PUBLIC ${XREAL_AIR_INCLUDE_DIR}
)

target_link_libraries(xrealAirDebug
		${XREAL_AIR_LIBRARY}
)
