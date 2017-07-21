#.rst:
# FindJPEG_Turbo
# --------
#
# Find JPEG_Turbo
#
# Find the JPEG_Turbo includes and library This module defines
#
# ::
#
#   JPEG_Turbo_INCLUDE_DIR, where to find jpeglib.h, etc.
#   JPEG_Turbo_LIBRARIES, the libraries needed to use JPEG_Turbo.
#   JPEG_Turbo_FOUND, If false, do not try to use JPEG_Turbo.
#
# also defined, but not for general use are
#
# ::
#
#   JPEG_Turbo_LIBRARY, where to find the JPEG_Turbo library.

#=============================================================================
# Copyright 2001-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

find_path(JPEG_Turbo_INCLUDE_DIR turbojpeg.h)

set(JPEG_Turbo_NAMES ${JPEG_Turbo_NAMES} turbojpeg)
find_library(JPEG_Turbo_LIBRARY NAMES ${JPEG_Turbo_NAMES} )

# handle the QUIETLY and REQUIRED arguments and set JPEG_FOUND to TRUE if
# all listed variables are TRUE
#include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(JPEG_Turbo DEFAULT_MSG JPEG_Turbo_LIBRARY JPEG_Turbo_INCLUDE_DIR)

if(JPEG_Turbo_FOUND)
  set(JPEG_Turbo_LIBRARIES ${JPEG_Turbo_LIBRARY})
endif()

# Deprecated declarations.
set (NATIVE_JPEG_Turbo_INCLUDE_PATH ${JPEG_Turbo_INCLUDE_DIR} )
if(JPEG_Turbo_LIBRARY)
  get_filename_component (NATIVE_JPEG_Turbo_LIB_PATH ${JPEG_Turbo_LIBRARY} PATH)
endif()
# message(STATUS "JPEG_Turbo_INCLUDE_DIR ${JPEG_Turbo_INCLUDE_DIR} JPEG_Turbo_LIBRARY ${JPEG_Turbo_LIBRARY} JPEG_Turbo_NAMES ${JPEG_Turbo_NAMES} JPEG_Turbo_FOUND ${JPEG_Turbo_FOUND}")
mark_as_advanced(JPEG_Turbo_LIBRARY JPEG_Turbo_INCLUDE_DIR )
