#.rst:
# FindTurboJPEG
# --------
#
# Find TurboJPEG
#
# Find the TurboJPEG includes and library This module defines
#
# ::
#
#   Turbo_JPEG_INCLUDE_DIR, where to find jpeglib.h, etc.
#   Turbo_JPEG_LIBRARIES, the libraries needed to use Turbo_JPEG.
#   Turbo_JPEG_FOUND, If false, do not try to use Turbo_JPEG.
#
# also defined, but not for general use are
#
# ::
#
#   Turbo_JPEG_LIBRARY, where to find the Turbo_JPEG library.

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

find_path(Turbo_JPEG_INCLUDE_DIR turbojpeg.h)

set(Turbo_JPEG_NAMES ${Turbo_JPEG_NAMES} turbojpeg)
find_library(Turbo_JPEG_LIBRARY NAMES ${Turbo_JPEG_NAMES} )

# handle the QUIETLY and REQUIRED arguments and set JPEG_FOUND to TRUE if
# all listed variables are TRUE
#include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Turbo_JPEG DEFAULT_MSG Turbo_JPEG_LIBRARY Turbo_JPEG_INCLUDE_DIR)

if(Turbo_JPEG_FOUND)
  set(Turbo_JPEG_LIBRARIES ${Turbo_JPEG_LIBRARY})
endif()

# Deprecated declarations.
set (NATIVE_Turbo_JPEG_INCLUDE_PATH ${Turbo_JPEG_INCLUDE_DIR} )
if(Turbo_JPEG_LIBRARY)
  get_filename_component (NATIVE_Turbo_JPEG_LIB_PATH ${Turbo_JPEG_LIBRARY} PATH)
endif()
message(STATUS "Turbo_JPEG_INCLUDE_DIR ${Turbo_JPEG_INCLUDE_DIR} Turbo_JPEG_LIBRARY ${Turbo_JPEG_LIBRARY} Turbo_JPEG_NAMES ${Turbo_JPEG_NAMES} Turbo_JPEG_FOUND ${Turbo_JPEG_FOUND}")
mark_as_advanced(Turbo_JPEG_LIBRARY Turbo_JPEG_INCLUDE_DIR )
