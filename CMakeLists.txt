# Software License Agreement (BSD License)
# 
# Point Cloud Library (PCL) - www.pointclouds.org
# Copyright (c) 2009-2012, Willow Garage, Inc.
# Copyright (c) 2012-, Open Perception, Inc.
# Copyright (c) CodeShop B.V. 2016-
# Copyright (c) 2017-, Stichting Centrum Wiskunde en Informatica.
# 
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met: 
# 
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder(s) nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 2.8.10 FATAL_ERROR)
project(CWI_PCL_CODEC)
# -- From: https://stackoverflow.com/questions/16398937/cmake-and-finding-other-projects-and-their-dependencies
# set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
# set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

# if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
#   message(SEND_ERROR "In-source builds are not allowed.")
# endif ()

# set(CMAKE_VERBOSE_MAKEFILE ON)
# set(CMAKE_COLOR_MAKEFILE   ON)

# You can set this to '1' if you like a lot of CMake messages, e.g. to find a bug
set (CMAKELISTS_DEBUG 0
        CACHE INTERNAL "${PROJECT_NAME}: CMakeLists.txt Debug Flag" FORCE)

set(SUBSYS_NAME cwi_pcl_codec)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
if(CMAKELISTS_DEBUG)
  message("CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")
  set(QUIET_FLAG "")
else(CMAKELISTS_DEBUG)
  set(QUIET_FLAG QUIET)
endif(CMAKELISTS_DEBUG)
find_package(PCL 1.8 REQUIRED ${QUIET_FLAG} COMPONENTS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition)
if(NOT PCL_FOUND)
  message(ERROR "PCL >= 1.8 not found.")
endif(NOT PCL_FOUND)

if(NOT PCL_ALL_IN_ONE_INSTALLER)
  # search for 3rd party packages
  if(WIN32)
    # Remove 'lib' prefix for shared libraries on Windows
    set(CMAKE_SHARED_LIBRARY_PREFIX "")
    set(CMAKE_C_FLAGS "(/EHsc /UBOOST_NO_EXCEPTIONS")
    set(CMAKE_CXX_FLAGS "/EHsc")
    set(CMAKE_LD_FLAGS "-g -O0")
    # On Windows we have VTK 7.0
    find_package(VTK 7.0 REQUIRED ${QUIET_FLAG})
    if(NOT VTK_FOUND)
      message(ERROR "VTK >= 7.0 found.")
    endif(VTK_FOUND)
  else(WIN32)
    # enable debugging on Linux
    set(CMAKE_C_FLAGS "-g -O0")
    set(CMAKE_CXX_FLAGS "-g -O0")
    set(CMAKE_LD_FLAGS "-g -O0")
    # On Ubuntu 16.04 we have VTK 6.0, on MacOSX (brew) VTK 8.1
    find_package(VTK 6.0 ${QUIET_FLAG})
    if(NOT VTK_FOUND)
     find_package(VTK 6.1 ${QUIET_FLAG})
    endif(NOT VTK_FOUND)
    if(NOT VTK_FOUND)
     find_package(VTK 8.0 ${QUIET_FLAG})
    endif(NOT VTK_FOUND)
    if(NOT VTK_FOUND)
      find_package(VTK 8.1 REQUIRED)
    endif(NOT VTK_FOUND)
    if(NOT VTK_FOUND)
       message(ERROR "Neither VTK 6.0, 6.1, 8.0 nor 8.1 found.")
    endif(NOT VTK_FOUND)
  endif(WIN32)
endif(NOT PCL_ALL_IN_ONE_INSTALLER)

#include(${CMAKE_SOURCE_DIR}/cmake/pcl_find_boost.cmake)
set(BOOST_REQUIRED_MODULES system filesystem thread date_time iostreams chrono program_options)
find_package(Boost 1.47.0 ${QUIET_FLAG} REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES})
if(NOT Boost_FOUND)
  message(ERROR "Boost >= 1.48.0 not found.")
endif(NOT Boost_FOUND)
if(CMAKELISTS_DEBUG)
 message("1.Boost_LIBRARIES=" ${Boost_LIBRARIES})
endif(CMAKELISTS_DEBUG)
find_package(Eigen 4.3 ${QUIET_FLAG})
if (NOT Eigen_FOUND)
  message(ERROR "Eigen not found)")
endif(NOT Eigen_FOUND)
if(CMAKELISTS_DEBUG)
  message(STATUS "Eigen found (include: ${EIGEN_INCLUDE_DIRS}, version: ${EIGEN_VERSION})")
endif(CMAKELISTS_DEBUG)
find_package(JPEG_Turbo ${QUIET_FLAG} REQUIRED)
if(CMAKELISTS_DEBUG)
  message("2.Boost_LIBRARIES=" ${Boost_LIBRARIES})
endif(CMAKELISTS_DEBUG)
if(NOT JPEG_Turbo_FOUND)
 message(ERROR "TurboJPEG not found.")
endif(NOT JPEG_Turbo_FOUND)

if(EXISTS "${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}/pcl/io/impl/entropy_range_coder.hpp")
    set(PCL_INSTALLED ON CACHE INTERNAL "PCL Installed" FORCE)
endif()

# Expose public includes and libraries to subprojects
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                 ${JPEG_Turbo_INCLUDE_DIR}
                                 ${VTK_INCLUDE_DIR}
                                 ${Boost_INCLUDE_DIR}
                                 ${PCL_INCLUDE_DIRS}
                                 ${EIGEN_INCLUDE_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
if(CMAKELISTS_DEBUG)
  message("${SUBSYS_NAME}-${PROJECT_NAME}_INCLUDE_DIRS: " ${${PROJECT_NAME}_INCLUDE_DIRS})
  message("3.Boost_LIBRARIES=" ${Boost_LIBRARIES})
endif(CMAKELISTS_DEBUG)
set(${PROJECT_NAME}_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/lib
                                 ${VTK_LIBRARY_DIR}
                                 ${Boost_LIBRARY_DIR}
                                 ${PCL_LIBRARY_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Library Directories" FORCE)
if(CMAKELISTS_DEBUG)
  message("${SUBSYS_NAME}_LIBRARY_DIRS: " ${${PROJECT_NAME}_LIBRARY_DIRS})
  message("${SUBSYS_NAME}_JPEG_Turbo_LIBRARY: " ${JPEG_Turbo_LIBRARY})
  message("${SUBSYS_NAME}_VTK_LIBRARIES:" ${VTK_LIBRARIES})
  message("${SUBSYS_NAME}_Boost_LIBRARIES: " ${Boost_LIBRARIES})
  message("${SUBSYS_NAME}_PCL_LIBRARIES: " ${PCL_LIBRARIES})
endif(CMAKELISTS_DEBUG)
set(${PROJECT_NAME}_LIBRARIES
                                 ${JPEG_Turbo_LIBRARY}
                                 ${VTK_LIBRARIES}
                                 ${Boost_LIBRARIES}
                                 ${PCL_LIBRARIES}
    CACHE INTERNAL "${PROJECT_NAME}: Libraries" FORCE)
if(CMAKELISTS_DEBUG)
  message("${SUBSYS_NAME}-${PROJECT_NAME}_LIBRARIES: " ${${PROJECT_NAME}_LIBRARIES})
endif(CMAKELISTS_DEBUG)
set(${PROJECT_NAME}_DEFINITIONS
                                 ${JPEG_Turbo_DEFINITIONS} -DWITH_JPEG8=1
                                 ${Boost_DEFINITIONS}
                                 ${PCL_DEFINITIONS}
    CACHE INTERNAL "${PROJECT_NAME}: Definitions" FORCE)
if(CMAKELISTS_DEBUG)
  message("${SUBSYS_NAME}-${PROJECT_NAME}_DEFINITIONS: " ${${PROJECT_NAME}_DEFINITIONS})
endif(CMAKELISTS_DEBUG)

add_definitions( ${${PROJECT_NAME}_DEFINITIONS})
add_subdirectory("jpeg_io")
add_subdirectory("cloud_codec_v2")
add_subdirectory("apps/evaluate_compression")
