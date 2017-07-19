# Copyright (c) 2017-, Stichting Centrum Wiskunde en Informatica (CWI).

cwi-pcl-codec
=============

This distribution contains a codec for encoding/decoding 3D Point Cloud data streams.   
The codec is described in detail in a journal paper, of which a preprint is available at: https://ir.cwi.nl/pub/24395.   
It is currently being reviwed by the MPEG working group 3DG-PCC on PointCloud Compression
(http://mpeg.chiariglione.org/standards/exploration/point-cloud-compression).

This version can be build on  Ubuntu 16.04 (64 bit) or Windows 32 bit, using pre-build PointCloudLibrary(PCL)
installers, or on many other systems by downloading and building PCL and its depencies.

This package contains of:

* codec software (cloud_codec_v2)
* auxiliary files needed (for using 'cmake'  and building 'jpeg_io')
* quality metrics
* evaluation library
* tools for testing and evalution of several aspects of this codec
* this installation instruction

To use it, several dependencies (Boost,Eigen,Flann,QHull,VTK and libjpeg-turbo) need to be installed:  

* for Ubuntu 16.04 by installing a number of Debian packages
* for Windows 7,8 and 10, most of this can be done using an all-in-one installer
* for all other supported systems by downloading, building and installing PCL 
  and its nessesary Third Party Package (TPP's) as described at:
  http://pointclouds.org/downloads -> 'Compiling from source'.

Installation
============

Easy Ubuntu 16.04 Install PCL 1.8.0 using binary packages:
----------------------------------------------------------

* On a clean Ubuntu 16.04 installation, start 'Terminal' and install the basic tools and 3rd party packages packages:  
   sudo apt-get install -y git build-essential linux-libc-dev cmake cmake-gui cmake cmake-gui libusb-1.0-0-dev libusb-dev libudev-dev mpi-default-dev openmpi-bin openmpi-common libflann1.8 libflann-dev libeigen3-dev libboost-all-dev libvtk6.2-qt libvtk6.2 libvtk6-dev libvtk6-qt-dev libqhull* libgtest-dev freeglut3-dev pkg-config libxmu-dev libxi-dev mono-complete qt-sdk openjdk-8-jdk openjdk-8-jre libopenni0 libopenni-sensor-pointclouds0  libopenni-dev libopenni-sensor-pointclouds-dev libproj-dev libjpeg-turbo8-dev
   
* Get the PCL-1.8 (Point CLoud Librarry) installer for Ubuntu 16.04 64-bit:  
wget https://www.dropbox.com/s/9llzm20pc4opdn9/PCL-1.8.0-Linux.deb   
(see also: 'https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/')   
(prior versions of PCL are not recommended, e.g. in PCL1.7 visualization does not work properly)

* Install PCL-1.8:
  sudo dpkg -i PCL-1.8.0-Linux.deb

* This installer has a bug, for which a patch is to be used: 'cd' to the directory 'cwi-pcl-codec'
  (where this file README.md was distributed), and type:
  (PATCH=$PWD/PCLConfig-Ubuntu16.04.patch;cd /;sudo patch -p1 < $PATCH)
  
* Start 'cmake-gui', specify the directory where this file is located in 'Where is the source code',
  another empty directory 'Where to build the binaries', and select 'Unix Makefiles' in the 'CMakeSetup'
  pop-up window. Click(tap) 'Configure', and 'Generate'.

Now the codec libraries and evaluation tools can be build by typing 'make' in the directory
that was specified in 'cmake-gui' to build the binaries.

Less easy install on Windows 7,8,10:
------------------------------------

* Install 'Visual Studio (2015)' and 'cmake-gui'

* Install PCL-1.8 and all 3rd party packages that it needs using its
   All-In-One Installer from 'http://unanancyowen.com/en/pcl18/':
  'https://1drv.ms/u/s!ApoY_0Ymu57sg5QkeGyAxxAmuI4j0g' (32 bit installer)

* Install 'libjpeg-turbo' from 'www.libjpeg-turbo.org':
  https://sourceforge.net/projects/libjpeg-turbo/files/1.5.1/libjpeg-turbo-1.5.1-gcc.exe
  (32 bit installer)

* Start 'cmake-gui', select for 'source code' the directory 'cwi-pcl-codec' (where this file README.md
  distributed), and for 'binaries' another directory

* Search for 'jpeg', for JPEG_INCLUDES specify 'C:libjpeg-turbo-gcc/include' and for 'JPEG_LIBRARY'
  'libjpeg-turbo-gcc/lib/turbojpeg'.
  Next select 'Configure' and 'Generate', and you'll find a Microsoft Visual Studio Solution
  in the directory that was specified for 'binaries'.
 
* Start Visual Studio with the 'Solution' file created in the prious paragraph and select 'Build->Build Solution'.

* After succesfull building, the program 'evaluate_compression.exe' can be found in the directory:
 'binaries'\apps\evaluate_compression\Debug.
  Before running, adapt the following environment variable:
  set path=%path%;C:\libjpeg-turbo-gcc\bin;C:\Program Files (x86)\OpenNI2\Tools 
 
  Suitable input files for the program can be downloaded from: http://vcl.iti.gr/reconstruction/
  Most of these data sets are huge; unpack some and specify the full directory path as an argument
  to the program:
  evaluate_compression --input_directories=<full path to directory with datafiles>


Not so easy (tedious, but not difficult) install PCL 1.8.0 from source: (all platforms):
-------------------------------------------------------------------------------

* Get PCL from github:
  git clone https://github.com/PointCloudLibrary/pcl
  cd pcl
  git branch 1.8.0

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
  -f [ --bb_expand_factor ] arg (=0.2)  bounding box expansion bto keep bounding box accross frames  
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

Apr.6, 2017, updated: Jun 19, 2017.   
Kees Blom (Kees.Blom@cwi.nl) CWI, Amsterdam, The Netherlands


