cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(road_quality_test)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(./src/include)
include_directories(./configs)

add_executable (road_rti src/road_rti.cpp)
target_link_libraries (road_rti ${PCL_LIBRARIES})
