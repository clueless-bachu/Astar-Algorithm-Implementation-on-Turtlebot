cmake_minimum_required(VERSION 3.5.1)
project (A_star)

set(CMAKE_CXX_STANDARD 11)
# Add project cmake modules to path.
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# Set build type
#set(CMAKE_BUILD_TYPE Debug)

# Import opencv 
find_package(OpenCV 3.3.1 REQUIRED)
#include opencv directories
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(astar Astar_diff.cpp)
target_link_libraries(astar ${OpenCV_LIBS})
