cmake_minimum_required (VERSION 3.8)


project ("sample")

set(CMAKE_CXX_STANDARD 11)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ../../include)
link_directories(${PCL_LIBRARY_DIRS} ../../build/Release)
add_definitions(${PCL_DEFINITIONS})
add_executable(sample main.cpp)


IF (WIN32)
	TARGET_LINK_LIBRARIES (sample 
		${PCL_LIBRARIES}
		${PROJECT_SOURCE_DIR}/../../build/Release/benewake_lidar_driver.lib
	)
ELSEIF (UNIX)
	TARGET_LINK_LIBRARIES (sample 
		${PCL_LIBRARIES}
		${PROJECT_SOURCE_DIR}/../../build/libbenewake_lidar_driver.so
	)
ENDIF ()