
Installation
============

To build the package 'cwi-pcl-codec', first the Point Cloud Library (PCL) (http://pointclouds.org)
and its dependencies need to be installed:  

* for Ubuntu 18.04 by installing 2 Debian packages ('gcc', 'cmake' and 'make' required):
  libpcl-devpcl and libturbojpeg0-dev using the package manger 'synaptic'

* for Windows 8 and 10, use the all-in-one installer for PCL-1.8.1 for your system and Visual Studio version
  at: http://unanancyowen.com/en/pcl181/. In addition, get and install 'libjpeg-turbo', from 'libjpeg-turbo.org'

* For MacOS 10.13.8 High Sierra type in a terminal window: brew install jpeg-turbo pcl

* for all other supported systems by downloading, building and installing PCL
  and its necessary Third Party Package (TPP's: Boost,Eigen,Flann,QHull,VTK and libjpeg-turbo) as described at:
  http://pointclouds.org/downloads -> 'Compiling from source'.


Ubuntu 18.04 Build & Install:
-----------------------------

* Start 'cmake-gui (>= 3.10)', specify the directory where this file is located in 'Where is the source code',
  another empty directory 'Where to build the binaries', and select 'Unix Makefiles' in the 'CMakeSetup'
  pop-up window. Click(tap) 'Configure', and 'Generate'.

Now the codec libraries and evaluation tools can be build by typing 'make' in the directory
that was specified in 'cmake-gui' to build the binaries.

 Windows 8,10 Build & Install:
-----------------------------------

* Install 'Visual Studio (2015)' and 'cmake-gui'
  Download source tarball for 'libjpeg-turbo' from 'www.libjpeg-turbo.org':
  https://sourceforge.net/projects/libjpeg-turbo/files/1.5.3/libjpeg-turbo-1.5.3.tar.gz/download
  Unpack the tarball and start 'cmake_gui', select for 'source code' directory the top-level directory
  of 'libturbo-jpeg' (contains 'CMakelists.txt'), and for 'binaries' another directory, click 'Configure' and 'Generate'.
  Now in your 'binaries' directory open the file 'libjpeg-turbo-1.5.3.sln' with Visual Studio 2015.
  In the Solution Explorer click Project 'INSTALL'.
  Select Build->Build Solution, if this is successful  select 'Build->Build INSTALL'.
  By default this installs the include files and libraries libraries in 'C:\libjpeg-turbo\include' and
  'C:\libjpeg-turbo\lib'

* Next start 'cmake-gui', select for 'source code' the directory 'cwi-pcl-codec' (where this file INSTALL.md
  is located), and for 'binaries' another (empty) directory

* For JPEG_INCLUDES specify 'C:/libjpeg-turbo/include' and for 'JPEG_LIBRARY'
  'C:/libjpeg-turbo/lib/turbojpeg-static.lib'.
  Next select 'Configure' and 'Generate', and you'll find a Microsoft Visual Studio Solution
  in the directory that was specified for 'binaries'.

* Start Visual Studio with the 'Solution' file created in the previous paragraph and select 'Build->Build Solution'.

* After successful building, the program 'evaluate_compression.exe' can be found in the directory:
 'binaries'\apps\evaluate_compression\Debug.
  Before running, adapt the following environment variable:
  set path=%path%;C:\libjpeg-turbo-gcc\bin;C:\Program Files (x86)\OpenNI2\Tools

  Suitable input files for the program can be downloaded from: http://vcl.iti.gr/reconstruction/
  Most of these data sets are huge; unpack some and specify the full directory path as an argument
  to the program:
  evaluate_compression --input_directories=<full path to directory with datafiles>

Other platforms: install PCL 1.8.1 from source:
----------------------------------------------------------------------------------------

* Get PCL source code from 'https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.8.1'
  (note that the source code in the PCL development tree is not compatible with this package).

* Get 3rd party packages:
  follow the instructions in: http://pointclouds.org/documentation/tutorials/compiling_pcl_dependencies_windows.php
  or http://pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php#compiling-pcl-macosx
  Be aware that the version numbers of some 3rd party packages are outdated and should match those used in the
  'apt-install' commands above.

* Use 'cmake-gui (>= 3.10)' to configure and generate the files for building each of the additional libraries and excutables in these package;
  build and install each of the libraries using the build system selected by 'cmake'

* Use 'cmake-gui (>= 3.10)' to configure and generate the files for building PCL;
  build and install the libraries using the build system selected by 'cmake'

* Download libjpeg-turbo  https://sourceforge.net/projects/libjpeg-turbo/files/1.5.3/libjpeg-turbo-1.5.3.tar.gz/download
  Use: 'configure; make install' to install the libraries

* Use 'cmake-gui (>= 3.10)' to configure and generate the files for building cwi-pcl-codec;
  build and install the libraries using the build system selected by 'cmake'
