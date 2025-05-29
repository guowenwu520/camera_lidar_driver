cmake_minimum_required(VERSION 3.8)
project(CollectionServer)

set(CMAKE_CXX_STANDARD 11)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

link_directories(
    ${PROJECT_SOURCE_DIR}/benewakeLidar
)

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(CollectionServer
    ${SRC_FILES}
)


if(WIN32)
    target_link_libraries(CollectionServer
        ${PROJECT_SOURCE_DIR}/benewakeLidar/benewake_lidar_driver.lib
    )
elseif(UNIX)
    target_link_libraries(CollectionServer
        benewake_lidar_driver 
    )
endif()

