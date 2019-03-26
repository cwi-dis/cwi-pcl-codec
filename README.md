> Copyright (c) 2017-, Stichting Centrum Wiskunde en Informatica (CWI).

# cwi-pcl-codec

This distribution contains a codec for encoding/decoding 3D Point Cloud data streams, and a toolset for its objective evaluation.   
The codec is described in detail in a journal paper:
> _(R. Mekuria, K. Blom, and P. Cesar, "Design, Implementation and Evaluation of a Point Cloud Codec for Tele-Immersive Video," IEEE Transactions on Circuits and Systems for Video Technology, 27(4): 828 -842, 2017_

of which a preprint is available at: <https://ir.cwi.nl/pub/24395>.   

The codec has served as the software to generate the anchors for the Call for Proposals for Point Cloud Compression by  the MPEG working group 3DG-PCC on PointCloud Compression
<http://mpeg.chiariglione.org/standards/exploration/point-cloud-compression>.

This version can be build on  MacOSX 10.12 or later (using Homebrew), Ubuntu 18.04 (64 bit) (using 'apt') or Windows 10 64bit (using pre-build PointCloudLibrary(PCL)
installers), or on many other systems by downloading and building PCL and its dependencies.

This package contains:

* codec software (cloud_codec_v2)
* auxiliary files needed (for using 'cmake'  and building 'jpeg_io')
* quality metrics
* evaluation library
* tools for testing and evaluation of several aspects of this codec
* installation instructions
* an API library that conforms to the *cwipc* API.

To use it, several dependencies need to be installed:  


* Boost
* Eigen
* Flann
* QHull
* VTK
* libjpeg-turbo

## Installation

### OSX instructions

- You need XCode.
- Install Homebrew, from <https://brew.sh>
- Install a few dependencies needed by our software and some of the third party libraries:

  ```
  brew install cmake
  brew install jpeg-turbo
  brew unlink jpeg
  brew link --force jpeg-turbo
  brew install pcl
  ```
  
  - The `brew unlink` and `brew link` are needed to install *jpeg-turbo* in stead of the normal jpeg library. Brew prefers not to do this, but that may lead to problems with some parts of the program linking to normal libjpeg and others to jpeg-turbo and the two getting into each others way.
- Download and the *cwipc_util* helper library from <https://github.com/cwi-dis/cwipc_util>.
	- By the time you read this there may be installers, otherwise download the source and build and install according to the instructions there.
- Build the compression library. Its cmakefiles need a little help, because _libjpeg-turbo_ isn't installed system-wide (because of name conflict with the normal libjpeg):
	
	```
	mkdir build-makefiles
	cd build-makefiles
	cmake ..
	make
	make test # optional
	make install # optional
	```
	
	- Alternatively you can _cmake_ for Xcode projects in stead of Makefiles: append `-G Xcode` to the _cmake_ command line above.
	
### Linux instructions

- These instructions are for Ubuntu 18.04.
- Install cmake, pcl and other dependencies:

  ```
  sudo apt-get install cmake
  sudo apt-get install libturbojpeg0-dev
  sudo apt-get install libpcl-dev libpcl-common1.8 libpcl-io1.8
  ```
  - This installs pcl 1.8, because 1.9 isn't available easily for Ubuntu 18.04.
- Download and the *cwipc_util* helper library from <https://github.com/cwi-dis/cwipc_util>.
	- By the time you read this there may be installers, otherwise download the source and build and install according to the instructions there.
- in the codec directory (this project) create a build subdirectory and build everything:

	```
	cmake ..
	make
	make test
	make install
	```

### Windows instructions

- Install Windows 10-64
- Install Visual Studio Community Edition
- Install Notepad++
- Install Git (checkout as-is, commit as-is for Unix newlines)
- Install CMake
- Install PCL 1.9.1 using the all-in-one installer.
	- Project homepage is at <https://github.com/PointCloudLibrary/pcl>, go to release, pick the latest 1.9 one, download the 64bit all-in-one. 
	- Currently that is <https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.9.1/PCL-1.9.1-AllInOne-msvc2017-win64.exe>
- Download libjpeg-turbo via <https://libjpeg-turbo.org/>. Get the windows 64bit binary installer.
- Download and the *cwipc_util* helper library from <https://github.com/cwi-dis/cwipc_util>.
	- By the time you read this there may be installers, otherwise download the source and build and install according to the instructions there.
- Create a `build` subdirectory of the source directory (of this repo).
- Configure and build the cwi codec repo with cmake and Visual Studio, in one of two ways:
	- Using the GUI:
		- Configure the cwi codec repository using cmake-gui.
		- Generate VS projects.
		- Open solution in Visual Studio, build.
	- Using the command line:

		```
		cmake .. -G "Visual Studio 15 2017 Win64"
		cmake --build . --config Release
		cmake --build . --config Release --target RUN_TESTS
		cmake --build . --config Release --target INSTALL
		```
		
- To make things run I had to add the following directories to system environment variable PATH:
	- `c:\Program Files\PCL 1.9.1\bin`
	- `c:\Program Files\OpenNI2\Tools`
	- `c:\libjpeg-turbo64\bin`

## Test programs

Three test programs are included:

- `cwipc_encode` compresses a `.ply` pointcloud file to a `.cwicpc` compressed pointcloud file.
- `cwipc_decode` decompresses a `.cwicpc` compresssed file to a `.ply` pointcloud file.
- `evaluate_compression` allows for evaluation of the algorithm, see below.

## Running the evaluation program

The following arguments are recognized by the program `evaluate_compression`:  
(long version arguments without '--' can also be put in a file 'parameter_config.txt' in the working directory or its parent)

*  -h [ --help ] 
	*  produce help message  
*  -K [ --K_outlier_filter ] arg (=0)
    * K neighbours for radius outlier filter   
*  --radius arg (=0.01) 
    * radius outlier filter, maximum radius  
*  -g [ --group_size ] arg (=0)
    * maximum number of files to be compressed together (0=read all files, then en(de)code 1 by 1)  
*  -f [ --bb_expand_factor ] arg (=0.2)
    * bounding box expansion to keep bounding box equal accross frames  
*  -a [ --algorithm ] arg (=V2) 
     * compression algorithm ('V1' or 'V2')  
*  -i [ --input_directories ] arg
      * Directory containing supported files (.pcd or .ply)  
*  -o [ --output_directory ] arg
      *  Directory to store decompressed pointclouds (.ply)  
*  -s [ --show_statistics ] [=arg(=1)] (=0)
      * gather and show a bunch of releavant statistical data  
*  -v [ --visualization ] [=arg(=1)] (=0)
      * show both original and decoded PointClouds graphically  
*  -p [ --point_resolution ] arg (=0.2)
       * XYZ resolution of point coordinates  
*  -r [ --octree_resolution ] arg (=0.2)
       * voxel size  
*  -b [ --octree_bits ] arg (=11)
       * octree resolution (bits)  
*  -c [ --color_bits ] arg (=8) 
       * color resolution (bits)  
*  -e [ --enh_bits ] arg (=0) 
       * bits to code the points towards the center  
*  -t [ --color\_coding\_type ] arg (=1)
       * pcl=0,jpeg=1 or graph transform  
*  -m [ --macroblock_size ] arg (=16)
       * size of macroblocks used for predictive frame (has to be a power of 2)  
*  --keep_centroid  arg (=0) 
       * keep voxel grid positions or not  
*  --create_scalable arg (=0) 
       * create scalable bitstream (not yet implemented)  
*  --do\_connectivity_coding arg (=0) 
       * connectivity coding (not yet implemented)  
*  --icp\_on_original arg (=0)
       * icp\_on_original  
*  -q [ --jpeg_quality ] arg (=0) 
       * jpeg quality parameter  
*  -d [ --do\_delta_coding ] arg (=0) 
       * use delta (predictive) en(de)coding  
*  --do\_quality_computation arg (=0)
       * compute quality of en(de)coding  
*  --do\_icp\_color_offset arg (=0) 
       *  do color offset en(de)coding on predictive frames  
*  -j [ --num_threads ] arg (=1) 
       *  number of parallel threads (1=default, single  thread, no parallel execution)  
*  --intra\_frame\_quality_csv arg (=intra\_frame\_quality.csv)
       * intra frame coding quality results filename (.csv file)  
*  --predictive\_quality_csv arg
       * (=predictive_quality.csv) predictive coding quality results file name (.csv file)  
*  --debug_level arg (=0)
       *  debug print level (0=no debug print, 3=all debug print)  

The precise meanings of these parameters are explained in the journal paper mentioned above.

Apr.6, 2017, updated: Jun 25, 2017, Apr. 23, 2018 and Mar.19, 2019.

Kees Blom (Kees.Blom@cwi.nl) CWI, Amsterdam, The Netherlands


