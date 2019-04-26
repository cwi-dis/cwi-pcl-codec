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
project(cwipc_codec)

include(CTest)
include(GNUInstallDirs)

# Ensure all executable, dll/so and link-library outputs end up in the same directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

#
# Find cwipc_util
#
find_package(cwipc_util REQUIRED)
include_directories(${cwipc_util_INCLUDE_DIRS})
link_directories(${cwipc_util_LIBRARY_DIRS})

set(SUBSYS_NAME cwi_pcl_codec)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/;${CMAKE_MODULE_PATH}")
set(QUIET_FLAG QUIET)

# Use PCL 1.9 (preferred) or 1.8
find_package(PCL 1.9 ${QUIET_FLAG} COMPONENTS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition)
if(NOT PCL_FOUND)
	find_package(PCL 1.8 REQUIRED ${QUIET_FLAG} COMPONENTS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition)
endif(NOT PCL_FOUND)

find_package(Boost 1.47.0 ${QUIET_FLAG} REQUIRED COMPONENTS system filesystem thread date_time iostreams chrono program_options)
find_package(Eigen 4.3 ${QUIET_FLAG} REQUIRED)
find_package(JPEG_Turbo ${QUIET_FLAG} REQUIRED)

# Expose public includes and libraries to subprojects
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                 ${JPEG_Turbo_INCLUDE_DIR}
                                 ${Boost_INCLUDE_DIR}
                                 ${PCL_INCLUDE_DIRS}
                                 ${EIGEN_INCLUDE_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

set(${PROJECT_NAME}_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/lib
                                 ${JPEG_Turbo_LIBRARY_DIR}
                                 ${Boost_LIBRARY_DIR}
                                 ${PCL_LIBRARY_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Library Directories" FORCE)

set(${PROJECT_NAME}_LIBRARIES
                                 ${JPEG_Turbo_LIBRARY}
                                 ${VTK_LIBRARIES}
                                 ${Boost_LIBRARIES}
                                 ${PCL_LIBRARIES}
    CACHE INTERNAL "${PROJECT_NAME}: Libraries" FORCE)

set(${PROJECT_NAME}_DEFINITIONS
                                 ${JPEG_Turbo_DEFINITIONS} -DWITH_JPEG8=1
                                 ${Boost_DEFINITIONS}
                                 ${PCL_DEFINITIONS}
    CACHE INTERNAL "${PROJECT_NAME}: Definitions" FORCE)

add_definitions( ${${PROJECT_NAME}_DEFINITIONS})
add_subdirectory("cloud_codec_v2")
add_subdirectory("src")
add_subdirectory("apps/evaluate_compression")
add_subdirectory("apps/cwipc_encode")
add_subdirectory("apps/cwipc_decode")
add_subdirectory("apps/cwipc_downsample")
add_subdirectory("apps/cwipc_tilefilter")
add_subdirectory("python")
add_subdirectory("tests")
