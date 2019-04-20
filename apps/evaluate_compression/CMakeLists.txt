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
set(SUBSYS_NAME evaluate_compression)
set(SUBSYS_DESC "Evaluation program for Point Cloud Compression V2 (using JPEG compression")


# Set the following to 1 to enable the -v (visualisation) flag to evaluate_compression. Requires vtk.
set(ENABLE_VISUALIZATION 0)
if(ENABLE_VISUALIZATION)
	set(VISUALIZATION_OPTIONAL visualization)
	set(VISUALIZATION_DEFINITIONS -DWITH_VTK)
endif()

if(NOT PCL_ALL_IN_ONE_INSTALLER)
  # search for 3rd party packages
  if(WIN32)
    # Remove 'lib' prefix for shared libraries on Windows
    set(CMAKE_SHARED_LIBRARY_PREFIX "")
    set(CMAKE_C_FLAGS "(/EHsc /UBOOST_NO_EXCEPTIONS")
    set(CMAKE_CXX_FLAGS "/EHsc")
    set(CMAKE_LD_FLAGS "-g -O0")
    if(ENABLE_VISUALIZATION)
		# On Windows we have VTK 7.0
		find_package(VTK 7.0 REQUIRED ${QUIET_FLAG})
		if(NOT VTK_FOUND)
		  message(ERROR "VTK >= 7.0 found.")
		endif(VTK_FOUND)
	endif()
  else(WIN32)
    # enable debugging on Linux
    set(CMAKE_C_FLAGS "-g -O0")
    set(CMAKE_CXX_FLAGS "-g -O0")
    set(CMAKE_LD_FLAGS "-g -O0")
    if(ENABLE_VISUALIZATION)
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
	endif()
  endif(WIN32)
endif(NOT PCL_ALL_IN_ONE_INSTALLER)

set(${PROJECT_NAME}_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/pcl/apps/${SUBSYS_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/pcl/apps/${SUBSYS_NAME}/impl
  ${${PROJECT_NAME}_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIR}
  CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
if(CMAKELISTS_DEBUG)
  message("${SUBSYS_NAME}-${PROJECT_NAME}_INCLUDE_DIRS: " ${${PROJECT_NAME}_INCLUDE_DIRS})
endif(CMAKELISTS_DEBUG)

set(srcs
  "src/${SUBSYS_NAME}.cpp"
  "src/quality_metrics.cpp"
)
set(LIB_NAME pcl_${SUBSYS_NAME})
set(incs
    "include/pcl/apps/${SUBSYS_NAME}/${SUBSYS_NAME}.h"
    "include/pcl/apps/${SUBSYS_NAME}/quality_metrics.h"
)
set(impl_incs
   "include/pcl/apps/${SUBSYS_NAME}/impl/${SUBSYS_NAME}_impl.hpp"
   "include/pcl/apps/${SUBSYS_NAME}/impl/quality_metrics_impl.hpp"
   )
include_directories(${${PROJECT_NAME}_INCLUDE_DIRS})

set(${PROJECT_NAME}_LIBRARY_DIRS
  ${CMAKE_CURRENT_BINARY_DIR}
  ${${PROJECT_NAME}_LIBRARY_DIRS}
  ${VTK_LIBRARY_DIR}
  CACHE INTERNAL "${PROJECT_NAME}: Library Directories" FORCE)
if(CMAKELISTS_DEBUG)
  message("${SUBSYS_NAME}-${PROJECT_NAME}_LIBRARY_DIRS: " ${${PROJECT_NAME}_LIBRARY_DIRS})
endif(CMAKELISTS_DEBUG)

set(${PROJECT_NAME}_LIBRARIES
#  ${LIB_NAME}
  ${${PROJECT_NAME}_LIBRARIES}  
  ${VTK_LIBRARIES}
  CACHE INTERNAL "${PROJECT_NAME}:Libraries" FORCE)
if(CMAKELISTS_DEBUG)
  message("${SUBSYS_NAME}-${PROJECT_NAME}_LIBRARIES: " ${${PROJECT_NAME}_LIBRARIES})
endif(CMAKELISTS_DEBUG)

add_definitions(${VISUALIZATION_DEFINITIONS})

add_executable(${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
target_link_libraries(${SUBSYS_NAME} ${${PROJECT_NAME}_LIBRARIES})

add_test(NAME ${SUBSYS_NAME} COMMAND ${SUBSYS_NAME} -o ${SUBSYS_NAME}_output ${CMAKE_SOURCE_DIR}/tests/fixtures/input )
