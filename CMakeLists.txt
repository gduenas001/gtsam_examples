# This file shows how to build and link a user project against GTSAM using CMake
###################################################################################
# To create your own project, replace "example" with the actual name of your project
cmake_minimum_required(VERSION 3.0)
project(gtsam_sim CXX C)


find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode

# Find GTSAM, either from a local build, or from a Debian/Ubuntu package.
find_package(GTSAM REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")

add_executable(main
  			   scenario_isam.cpp
  			   helpers.cpp
  			   RangeBearingFactorMap.cpp
  			   parser.cpp
  			   Counters.cpp
  			   add_factors.cpp
  			   Solution.cpp
  			   calculate_LIR.cpp
  			   ./third_party/easyloggingpp/src/easylogging++.cc
)


# By using CMake exported targets, a simple "link" dependency introduces the
# include directories (-I) flags, links against Boost, and add any other
# required build flags (e.g. C++11, etc.)
target_link_libraries(main PRIVATE gtsam gtsam_unstable)

add_executable(test
			   test.cpp
			   ./third_party/easyloggingpp/src/easylogging++.cc
)

