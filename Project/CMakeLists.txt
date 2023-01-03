cmake_minimum_required(VERSION 3.22)
project(marginalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(GTSAM REQUIRED)
set(GTSAM_LIBRARIES gtsam)

add_executable(${PROJECT_NAME} main.cpp file_utils.cpp relation.cpp node.cpp)

target_link_libraries(${PROJECT_NAME} ${GTSAM_LIBRARIES})
