cmake_minimum_required(VERSION 3.22)
project(marginalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(GTSAM REQUIRED)
set(GTSAM_LIBRARIES gtsam)

find_package(Python3 COMPONENTS Interpreter Development NumPy)
include_directories(${Python3_INCLUDE_DIRS} include)


add_executable(${PROJECT_NAME} main.cpp string_utils.cpp file_utils.cpp)

target_link_libraries(${PROJECT_NAME} ${GTSAM_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES})