# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project ("benewake_lidar_driver")

SET(SRC include/benewake_hornx2_driver.h 
		include/benewake_lidar_driver.h 
		include/benewake_common.h
		include/benewake_dcsp.h
		include/benewake_dsop.h
		include/benewake_mdop.h
        include/benewake_protocol.h
		include/benewake_udp.h
		include/msg_queue.h
		include/benewake_tables.h
		src/benewake_hornx2_driver.cpp
		src/benewake_lidar_driver.cpp
        src/benewake_dcsp.cpp
        src/benewake_dsop.cpp
        src/benewake_mdop.cpp
        src/benewake_protocol.cpp
        src/benewake_udp.cpp)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread")


LINK_DIRECTORIES(
)

ADD_DEFINITIONS (
	-DBENEWAKE_DLL_EXPORT
)

INCLUDE_DIRECTORIES (
	${CMAKE_SOURCE_DIR}/include
)

ADD_LIBRARY (benewake_lidar_driver SHARED
	${SRC}
)

SET_TARGET_PROPERTIES (benewake_lidar_driver PROPERTIES PUBLIC_HEADER 
	"include/benewake_hornx2_driver.h;include/benewake_lidar_driver.h;include/benewake_common.h;include/benewake_dcsp.h;include/benewake_dsop.h;include/benewake_mdop.h;include/benewake_protocol.h;include/benewake_udp.h;include/msg_queue.h;include/benewake_tables.h")

if(WIN32)
INSTALL (TARGETS benewake_lidar_driver
	PUBLIC_HEADER DESTINATION include
	RUNTIME DESTINATION lib
)
else()
INSTALL (TARGETS benewake_lidar_driver
	PUBLIC_HEADER DESTINATION include
	LIBRARY DESTINATION lib
)
endif()
