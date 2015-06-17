cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_learn)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (RANSAC_line_plane_sphere_fitting ../src/RANSAC_line_plane_sphere_fitting.cpp)
target_link_libraries (RANSAC_line_plane_sphere_fitting ${PCL_LIBRARIES})
