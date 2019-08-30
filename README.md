# Copyright (c) 2017-, Stichting Centrum Wiskunde en Informatica (CWI).

cwi-pcl-codec
=============

This distribution contains a codec for encoding/decoding 3D Point Cloud data streams, and a toolset for its objective evaluation.   
The codec is described in detail in a journal paper(R. Mekuria, K. Blom, and P. Cesar, "Design, Implementation and Evaluation of a Point Cloud Codec for Tele-Immersive Video," IEEE Transactions on Circuits and Systems for Video Technology, 27(4): 828 -842, 2017, of which a preprint is available at: https://ir.cwi.nl/pub/24395.   
The codec has served as the software to generate the anchors for the Call for Proposals for Point Cloud Compression by  the MPEG working group 3DG-PCC on PointCloud Compression
(http://mpeg.chiariglione.org/standards/exploration/point-cloud-compression).

This version can be build on  MacOSX 10.14.3 macOS Mojave (using Homebrew), Ubuntu 18.04 (64 bit) (using 'apt') or Windows 10 bit, using pre-build PointCloudLibrary(PCL)
installers, or on many other systems by downloading and building PCL and its dependencies.

This package contains:

* codec software (cloud_codec_v2)
* auxiliary files needed (for using 'cmake'  and building 'jpeg_io')
* quality metrics
* evaluation library
* tools for testing and evaluation of several aspects of this codec
* installation instructions

To use it, several dependencies (Boost,Eigen,Flann,QHull,VTK and libjpeg-turbo) need to be installed:  

* for Windows 10, most of this can be done using an all-in-one installer
* for macOS Mojave 14.3 using Homebrew.
* for Ubuntu 18.04 by installing a number of Debian packages
* for all other supported systems by downloading, building and installing PCL 
  and its necessary Third Party Package (TPP's) as described at:
  http://pointclouds.org/downloads -> 'Compiling from source'.

Installation
============

Mac OSX 10.14.3 Installation:
-----------------------------

* On a clean Mac OSX 10.14.3 installation install Xcode 10.1, Cmake 10.13.4 and Homebrew 2.0.6

* brew unlink jpeg
  brew install --HEAD jpeg-turbo
  brew install vtk pcl

* Start 'CMake', specify the directory where this file is located in 'Where is the source code',                            
  another empty directory 'Where to build the binaries', and select 'Xcode'  in the 'CMakeSetup'                        
  pop-up window. Then click(tap) 'Configure', and 'Generate'.
  Now an Xcode project should be generated in the directory specified for 'build the binaries':
  CWI-PCL-CODEC.xcodeproj.

* Start 'Xcode' and use it to open the new Xcode-project, select 'Project->Build'.
  The resulting application can be found in 'apps/evaluate_compression' and is self-documenting.

Installation on Windows 10 (older Windows versions won't work):
---------------------------------------------------------------

* Install 'Visual Studio (2017)' Compiler Version 15.9 or higher and 'cmake-gui' Version 3.13

* Install PCL-1.9.1 and all 3rd party packages that it needs using the
  All-In-One Installer from: 'https://github.com/PointCloudLibrary/pcl/releases'
  Select (if possible) 64-bit version (PointClouds can be huge data sets, too big for 32-bit addressing)

* Download 'libjpeg-turbo64' from: 'https://sourceforge.net/projects/libjpeg-turbo/files/2.0.2/libjpeg-turbo-2.0.2.tar.gz/download'

* Next start 'cmake-gui', select for 'source code' the directory in which 'libjpeg-turbo-2.0.2' was unpacked and
  for'binaries' another directory, tap 'Configure' and 'Generate'.
  Start 'Visual Studio (2017)', open 'libjpeg-turbo.sln', select the 'INSTALL' Project to build and install 'libjpeg-turbo-2.0.2'.

* Next start 'cmake-gui', select for 'source code' the directory 'cwi-pcl-codec' (where this file README.md
  distributed), and for 'binaries' another directory

* Search for 'jpeg', for JPEG_INCLUDES specify 'C:/libjpeg-turbo64/include' and for 'JPEG_LIBRARY'
  'C:/libjpeg-turbo64/lib/turbojpeg-static.lib'.
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

Ubuntu 18.04 Install PCL 1.8.1 using binary packages:
-----------------------------------------------------

* On a clean Ubuntu 18.04 installation, start 'Terminal' and install the Point Cloud Library (PCL) and its dependencies:
   sudo apt install pcl-tools libpcl-dev  libjpeg-turbo8-dev

  
* Start 'cmake-gui', specify the directory where this file is located in 'Where is the source code',
  another empty directory 'Where to build the binaries', and select 'Unix Makefiles' in the 'CMakeSetup'
  pop-up window. Click(tap) 'Configure', and 'Generate'.
  For JPEG_Turbo_INCLUDE_DIR and JPEG_Turbo_LIBRARY, select '/usr/include' and '/usr/lib/x86_64-linux-gnu/libjpeg.s§o'.

Now the codec libraries and evaluation tools can be build by typing 'make' in the directory
that was specified in 'cmake-gui' to build the binaries. The application can be found in 'apps/evaluate_compression'.

Not so easy (tedious, but not difficult) install PCL 1.9.1 from source: (all platforms):
----------------------------------------------------------------------------------------

* Get PCL source code from 'https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.8.0'
  (note that the source code in the PCL development tree is not compatible with this package).

* Get 3rd party packages:
  follow the instructions in: http://pointclouds.org/documentation/tutorials/compiling_pcl_dependencies_windows.php
  or http://pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php#compiling-pcl-macosx
  Be aware that the version numbers of some 3rd party packages are outdated and should match those used in the
  'apt-install' commands above.

* Use 'cmake-gui' to configure and generate the files for building PCL

* Use 'cmake-gui' to configure and generate the files for building the addional libraries and excutables in this package.

Running the evaluation program
==============================

The following arguments are recognized by the program 'evaluate_compression':  
(long version arguments without '--' can also be put in a file 'parameter_config.txt' in the working directory or its parent)

  -h [ --help ]                         produce help message  
  -K [ --K_outlier_filter ] arg (=0)    K neighbours for radius outlier filter   
  --radius arg (=0.01)                  radius outlier filter, maximum radius  
  -g [ --group_size ] arg (=0)          maximum number of files to be compressed together (0=read all files, then en(de)code 1 by 1)  
  -f [ --bb_expand_factor ] arg (=0.2)  bounding box expansion to keep bounding box equal accross frames  
  -a [ --algorithm ] arg (=V2)          compression algorithm ('V1' or 'V2')  
  -i [ --input_directories ] arg        Directory containing supported files (.pcd or .ply)  
  -o [ --output_directory ] arg         Directory to store decompressed pointclouds (.ply)  
  -s [ --show_statistics ] [=arg(=1)] (=0) gather and show a bunch of releavant statistical data  
  -v [ --visualization ] [=arg(=1)] (=0) show both original and decoded PointClouds graphically  
  -p [ --point_resolution ] arg (=0.2)  XYZ resolution of point coordinates  
  -r [ --octree_resolution ] arg (=0.2) voxel size  
  -b [ --octree_bits ] arg (=11)        octree resolution (bits)  
  -c [ --color_bits ] arg (=8)          color resolution (bits)  
  -e [ --enh_bits ] arg (=0)            bits to code the points towards the center  
  -t [ --color_coding_type ] arg (=1)   pcl=0,jpeg=1 or graph transform  
  -m [ --macroblock_size ] arg (=16)    size of macroblocks used for predictive frame (has to be a power of 2)  
  --keep_centroid  arg (=0)             keep voxel grid positions or not  
  --create_scalable arg (=0)            create scalable bitstream (not yet implemented)  
  --do_connectivity_coding arg (=0)     connectivity coding (not yet implemented)  
  --icp_on_original arg (=0)            icp_on_original  
  -q [ --jpeg_quality ] arg (=0)        jpeg quality parameter  
  -d [ --do_delta_coding ] arg (=0)     use delta (predictive) en(de)coding  
  --do_quality_computation arg (=0)     compute quality of en(de)coding  
  --do_icp_color_offset arg (=0)        do color offset en(de)coding on predictive frames  
  -j [ --num_threads ] arg (=1)         number of parallel threads (1=default, single  thread, no parallel execution)  
  --intra_frame_quality_csv arg (=intra_frame_quality.csv) intra frame coding quality results filename (.csv file)  
  --predictive_quality_csv arg (=predictive_quality.csv) predictive coding quality results file name (.csv file)  
  --debug_level arg (=0)                debug print level (0=no debug print, 3=all debug print)  

The precise meanings of these parameters are explained in the journal paper mentioned above.

Apr.6, 2017, updated: Jun 25, 2017, Apr. 23, 2018 and Mar.19, 2019.
   
Kees Blom (Kees.Blom@cwi.nl) CWI, Amsterdam, The Netherlands


