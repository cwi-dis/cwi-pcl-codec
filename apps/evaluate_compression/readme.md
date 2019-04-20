# evaluate_compression

This program can be used to test the compression algorithm and how it reacts to various parameter settings.

## Running the evaluation program

Suitable input files for the program can be downloaded from: <http://vcl.iti.gr/reconstruction/>
Most of these data sets are huge; unpack some and specify the full directory path as an argument
to the program:

```
evaluate_compression --input_directories=<full path to directory with datafiles>
```

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


