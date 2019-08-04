# This file shows how to build and link a user project against GTSAM using CMake
###################################################################################
# To create your own project, replace "example" with the actual name of your project
cmake_minimum_required(VERSION 3.0)
project(guillermo CXX)

# Find GTSAM, either from a local build, or from a Debian/Ubuntu package.
find_package(GTSAM REQUIRED)

add_executable(read_file_fg read_file_fg.cpp)

# By using CMake exported targets, a simple "link" dependency introduces the
# include directories (-I) flags, links against Boost, and add any other
# required build flags (e.g. C++11, etc.)
target_link_libraries(read_file_fg PRIVATE gtsam)


add_executable(scenario_isam scenario_isam.cpp)
target_link_libraries(scenario_isam PRIVATE gtsam)