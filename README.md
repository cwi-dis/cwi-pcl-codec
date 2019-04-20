> Copyright (c) 2017-2019, Stichting Centrum Wiskunde en Informatica (CWI).

# cwipc_codec

This distribution contains a compressor and decompressor for pointclouds. It uses the _cwipc_ abstract object to represent pointclouds.

It is a modified version of the cwi-pcl-codec distribution, from <http://github.com/cwi-dis/cwi-pcl-codec>. Both distributions share the same codec, described in the following journal paper:

> _(R. Mekuria, K. Blom, and P. Cesar, "Design, Implementation and Evaluation of a Point Cloud Codec for Tele-Immersive Video," IEEE Transactions on Circuits and Systems for Video Technology, 27(4): 828 -842, 2017_

## Installing

For use within VRtogether you can get pre-built zipfiles (or tgzfiles for Mac/Linux) from <https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_codec/releases>. Download the most recent release with a normal v_X_._Y_._Z_ name. You will also need the accompanying _cwipc\_util_ installer from 
<https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_util/releases>.

You also need to install a number of dependencies:

* Boost
* Eigen
* Flann
* QHull
* libjpeg-turbo

[![pipeline status](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_codec/badges/master/pipeline.svg)](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_codec/commits/master)

### Windows

- Install PCL 1.8 from <https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2017-win64.exe>. Make sure you select the "add to %PATH% for all users" option.
- Download libjpeg-turbo via <https://libjpeg-turbo.org/>. Get the windows 64bit binary installer. Install in the standard location `C:\libjpeg-turbo64`. Add `C:\libjpeg-turbo64\bin` to %PATH% for all users.
- Create a folder where you will install _all_ VRtogether DLLs and EXEs, for example `C:\vrtogether\installed`.
- Extract both the `cwipc_util_win1064_vX.Y.zip` and `cwipc_codec_win1064_vX.Y.zip` files into `c:\vrtogether`. This will create `bin`, `lib` and `include` folders inside the `C:\vrtogether\installed` folder.
- Add the `c:\vrtogether\installed\bin` folder to the `%PATH%` system environment variable.

### OSX

- Install _brew_, and then

  ```
  brew install pcl
  brew install jpeg-turbo
  brew unlink jpeg
  brew link --force jpeg-turbo
  ```

- Extract both gzip files into `/usr/local`:

  ```
  cd /usr/local
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  [sudo] tar xfv .../cwipc_codec_osx1012_vX.Y.tgz
  ```
  
### Ubuntu 18.04

- Install _PCL_ and _jpegturbo_ with 

  ```
  apt-get install libpcl-dev
  sudo apt-get install libturbojpeg0-dev
  ```

- Extract both gzip files into `/usr/local`:

  ```
  cd /usr/local
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  [sudo] tar xfv .../cwipc_codec_osx1012_vX.Y.tgz
  ```

## Building from source


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
- Download and install the *cwipc_util* helper library.
	- Use the *cwipc_util* binary distribution explained above, or 
	- clone from <https://github.com/cwi-dis/cwipc_util> and build it from source according to the instructions in there.
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

- Make sure you have the following on your development machine:
	- Install Windows 10-64
	- Install Visual Studio Community Edition
	- Install Notepad++
	- Install Git (checkout as-is, commit as-is for Unix newlines)
	- Install CMake
- Install PCL 1.9.1 using the all-in-one installer.
	- Project homepage is at <https://github.com/PointCloudLibrary/pcl>, go to release, pick the latest 1.9 one, download the 64bit all-in-one. 
	- Currently that is <https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.9.1/PCL-1.9.1-AllInOne-msvc2017-win64.exe>
- Download libjpeg-turbo via <https://libjpeg-turbo.org/>. Get the windows 64bit binary installer. Install in the standard location `C:\libjpeg-turbo64`.
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
		cmake .. -G "Visual Studio 15 2017 Win64" -DJPEG_Turbo_INCLUDE_DIR="C:\libjpeg-turbo64\include" -DJPEG_Turbo_LIBRARY="C:\libjpeg-turbo64\lib\jpeg.lib"
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
- `evaluate_compression` allows for evaluation of the algorithm, see [its readme file](apps/evaluate_compression/readme.md) for details.

The Python unittest [python/test\_cwipc\_codec.py](python/test_cwipc_codec.py) tests each individual feature and API call of the codec.