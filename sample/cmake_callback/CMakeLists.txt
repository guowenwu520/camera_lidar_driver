cmake_minimum_required (VERSION 3.8)

project ("sample")

set(CMAKE_CXX_STANDARD 11)

include_directories(../../include)
link_directories(../../build/Release)
add_definitions()
add_executable(sample main.cpp)


IF (WIN32)
	TARGET_LINK_LIBRARIES (sample 
		${PROJECT_SOURCE_DIR}/../../build/Release/benewake_lidar_driver.lib
	)
ELSEIF (UNIX)
	TARGET_LINK_LIBRARIES (sample 
		${PROJECT_SOURCE_DIR}/../../build/libbenewake_lidar_driver.so
	)
ENDIF ()