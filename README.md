# Copyright (c) 2017-, Stichting Centrum Wiskunde en Informatica (CWI).

cwi-pcl-codec
=============

This distribution contains a codec for encoding/decoding 3D Point Cloud data streams
and a toolset for its objective evaluation.   
It is described in detail in a journal paper (R. Mekuria, K. Blom, and P. Cesar,
"Design, Implementation and Evaluation of a Point Cloud Codec for Tele-Immersive Video,
" IEEE Transactions on Circuits and Systems for Video Technology, 27(4): 828 -842, 2017,
(preprint is available at: https://ir.cwi.nl/pub/24395).  
The codec has served as an anchor for the "Call for Proposals for Point Cloud Compression"
by  the MPEG working group 3DG-PCC on PointCloud Compression
(https://mpeg.chiariglione.org/standards/mpeg-i/point-cloud-compression/call-proposals-point-cloud-compression-v2).

The package contains:

* this file 'README.md'
* codec software (cloud_codec_v2)
* auxiliary files needed
* quality metrics library and app
* evaluation library
* tools for testing and evaluation of several aspects of this codec
* installation instruction ('INSTALL.md')

This version can be build on  Ubuntu 18.04 (64 bit), Windows 32/64 bit and MacOS 10.13.8 Hight Sierra,
using binary installers for the Point Cloud Library (PCL), or on many other systems by downloading
and building PCL and its dependencies.

For details, see the file 'INSTALL.md' inthis directory.

After building 2 applications will be available:

evaluate_compression - reads Pointcloud datafiles, compress/decompress each of them, output the resulting pointclouds,
		     visualizes and compares the original pointclouds with the encoded/decoded resulting pointclouds
		     by calculating an objective metrics, using a variety of algorithms.

Running evaluation_compression
==============================

The following arguments are recognized by the program 'evaluate_compression':  
(long version arguments without '--' can also be put in a file 'parameter_config.txt' in the working directory or its parent)

  -h [ --help ]                         produce help message  
  -K [ --K_outlier_filter arg ] (=0)    K neighbours for radius outlier filter   
  --radius (=0.01)                      radius outlier filter, maximum radius  
  -g [ --group_size arg ] (=0)          maximum number of files to be compressed together (0=read all files, then en(de)code 1 by 1)  
  -f [ --bb_expand_factor arg ] (=0.2)  bounding box expansion to keep bounding box equal accross frames  
  -a [ --algorithm  arg ] (=V2)         compression algorithm (='','V1','V2' or 'Delta')  
  -i [ --input_directories ] arg        Directory containing supported files (.pcd or .ply) (required)  
  -o [ --output_directory ] arg         Directory to store decompressed pointclouds (.ply)  
  -s [ --show_statistics arg ] (=0)     gather and show a bunch of releavant statistical data  
  -v [ --visualization arg ] (=0)       show both original and decoded PointClouds graphically in separate windows
  -p [ --point_resolution arg ] (=0.2)  XYZ resolution of point coordinates  
  -r [ --octree_resolution arg ] (=0.2) voxel size  
  -b [ --octree_bits arg ] (=11)        octree resolution (bits)  
  -c [ --color_bits arg ] (=8)          color resolution (bits)  
  -e [ --enh_bits ] arg ] (=0)          not implemented  
  -t [ --color_coding_type arg ] (=1)   pcl=0,jpeg=1  
  -m [ --macroblock_size arg ] (=16)    size of macroblocks used for predictive frame (has to be a power of 2)  
  --keep_centroid  arg (=0)             keep voxel grid positions or not  
# commented out options not implemented 
#--create_scalable arg (=0)            create scalable bitstream (not yet implemented)  
#--do_connectivity_coding arg (=0)     connectivity coding (not yet implemented)  
  --icp_on_original arg (=0)            icp_on_original  
  -j [ --jpeg_quality ] arg (=0)        jpeg quality parameter  
  -d [ --do_delta_coding ] arg (=0)     use delta (predictive) en(de)coding  
  -q [ --quality_method arg ](="NONE")  compute quality of en(de)coding using specified comma-separated methods:  
       			    		NONE,SELECT,BBALIGNED,TCSVT,MAX_NN,NORMALISED,BT709  
  --do_icp_color_offset arg (=0)        do color offset en(de)coding on predictive frames  
  -n [ --num_threads ] arg (=1)         number of parallel threads (1=default, single  thread, no parallel execution)  
  --intra_frame_quality_csv arg (=intra_frame_quality.csv) intra frame coding quality results filename (.csv file)  
  --predictive_quality_csv arg (=predictive_quality.csv) predictive coding quality results file name (.csv file)  
  --debug_level arg (=0)                debug print level (0=no debug print, 3=all debug print)  

The precise meanings of these parameters are explained in the journal paper mentioned above.

Available quality methods:  
NONE            - disable quality metric computation  
SELECT		- enable quality metric computation using default methods  
BBALIGNED       - use the differences of the bounding box aligned pointclouds before and after (de)compression  
                    instead of comparing the original pointcloud with the final resulting pointcloud
TCSVT           - use the same method as in the paper  
MAX_NN          - use the maximal nearest neighbour for the peak value in geometric PSNR  
NORMALISED      - scale all points in the pointclouds to be bounded by [0,1]  
BT709           - use the BT.709 algorithm instead of for RGB to YUV conversion


Running quality_metric
======================

The following arguments are recognized by the program 'quality_metric':  
(long version arguments without '--' can also be put in a file 'parameter_config.txt' in the working directory or its parent)


  -h [ --help ]                         produce help message  
  -a [ --fileA arg ]                    specify original pointcloud file (.ply or .pcd format)  
  -b [ --fileB arg ]                    specify degraded pointcloud file (.ply or .pcd format)  
  -q [ --quality_method arg ](="NONE")  compute quality of en(de)coding using specified comma-separated methods:
                          			    		NONE,SELECT,TCSVT,MAX_NN,NORMALISED,BT709


Apr.6, 2017, updated: Jun 25, 2017, Apr. 23, 2018, Aug.28, 2018 and Oct.10, 2018.   
Kees Blom (Kees.Blom@cwi.nl) CWI, Amsterdam, The Netherlands
