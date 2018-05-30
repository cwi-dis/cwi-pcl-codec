/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014- Centrum Wiskunde en Informatica
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder (s)  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
//
//  evaluate_compression.hpp
//  evaluate_compression
//
//  Created by Kees Blom on 01/06/16.
//
//
#ifndef evaluate_compression_hpp
#define evaluate_compression_hpp

#if defined(_OPENMP)
#include <omp.h>
#endif//defined(_OPENMP)
// c++ standard library
#include <fstream>
#include <vector>
#include <ctime> // for 'strftime'
#include <exception>

// boost library
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/parsers.hpp>
namespace po = boost::program_options;

// point cloud library
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>
#include <pcl/filters/radius_outlier_removal.h>

#define WITH_VTK 31
#ifdef  WITH_VTK
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <vtkRect.h>
#endif/*WITH_VTK*/
#ifdef WIN32
#include <direct.h>
#include <stdlib.h>
#include <stdio.h>
#endif//WIN32
#include <evaluate_comp.h>
#include <qual_metrics.h>

template<typename PointT>
class evaluate_comp_impl : evaluate_compression {
  // using boost::exception on errors
  public:
    evaluate_comp_impl (int argc, char** argv) : evaluate_compression (argc, argv), debug_level_ (3) {};
    bool evaluate_group(std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >& group, stringstream& compression_settings, std::ofstream& intra_frame_quality_csv, std::ofstream& predictive_quality_csv);

    // options handling
    void initialize_options_description ();
    bool get_options (int argc, char** argv);
    void assign_option_values ();
  
    po::options_description desc_;
    po::variables_map vm_;
    po::positional_options_description pod_;

    // preprocessing, encoding, decoding, quality assessment  void complete_initialization ();
    void complete_initialization ();

#ifdef  WITH_VTK
    typedef pcl::visualization::PCLVisualizer* ViewerPtr;
    ViewerPtr viewer_decoded_, viewer_delta_decoded_, viewer_original_;
    int viewer_decoded_window_position_[2];
    int viewer_delta_decoded_window_position_[2];
#endif//WITH_VTK
  
    void do_outlier_removal (std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >& pointcloud);
    pcl::io::BoundingBox do_bounding_box_normalization (std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >& pointcloud);
    void do_encoding (boost::shared_ptr<pcl::PointCloud<PointT> > point_cloud, stringstream* coded_stream, QualityMetric & achieved_quality);
    void do_decoding (stringstream* coded_stream, boost::shared_ptr<pcl::PointCloud<PointT> > pointcloud, QualityMetric & achieved_quality);
    void do_delta_encoding (boost::shared_ptr<pcl::PointCloud<PointT> > i_cloud, boost::shared_ptr<pcl::PointCloud<PointT> > p_cloud, boost::shared_ptr<pcl::PointCloud<PointT> > out_cloud, stringstream* i_stream, stringstream* p__stream, QualityMetric & quality_metric);
    void do_delta_decoding (stringstream* i_stream, stringstream* p_stream, boost::shared_ptr<pcl::PointCloud<PointT> > i_cloud, boost::shared_ptr<pcl::PointCloud<PointT> > out_cloud, QualityMetric & qualityMetric);
    void do_quality_computation (boost::shared_ptr<pcl::PointCloud<PointT> > & pointcloud, boost::shared_ptr<pcl::PointCloud<PointT> > &reference_pointcloud, QualityMetric & quality_metric);
    void do_output (std::string path, boost::shared_ptr<pcl::PointCloud<PointT> > pointcloud, QualityMetric & qualityMetric);
    // V1 (common) settings
    boost::shared_ptr<pcl::io::OctreePointCloudCompression<PointT> > encoder_V1_;
    boost::shared_ptr<pcl::io::OctreePointCloudCompression<PointT> > decoder_V1_;
    // V2 specific
    boost::shared_ptr<pcl::io::OctreePointCloudCodecV2<PointT> > encoder_V2_;
    boost::shared_ptr<pcl::io::OctreePointCloudCodecV2<PointT> > decoder_V2_;
  
    bool evaluate (); // TBD need catch exceptions
    void do_visualization (std::string id, boost::shared_ptr<pcl::PointCloud<PointT> > pointcloud);
    int debug_level_;
};


template<typename PointT>
void
evaluate_comp_impl<PointT>::initialize_options_description ()
{
  desc_.add_options ()
  ("help,h", "produce help message ")
  ("K_outlier_filter,K",po::value<int> ()->default_value (0), "K neighbours for radius outlier filter ")
  ("radius",po::value<double> ()->default_value (0.01), "radius outlier filter, maximum radius ")
  ("group_size,g",po::value<int> ()->default_value (0), "maximum number of files to be compressed together (0=read all files, then en(de)code 1 by 1)")
  ("bb_expand_factor,f", po::value<double> ()->default_value (0.20), "bounding box expansion to keep bounding box accross frames")
  ("algorithm,a",po::value<string> ()->default_value ("V2"), "compression algorithm ('V1' or 'V2')")
  ("input_directories,i", po::value<std::vector<std::string> > (), "Directory containing supported files (.pcd or .ply)")
  ("output_directory,o", po::value<std::string> ()->default_value (""), "Directory to store decompressed pointclouds (.ply)")
  ("show_statistics,s",po::value<bool> ()->default_value (false)->implicit_value (true), "gather and show a bunch of releavant statistical data")
  ("visualization,v",po::value<bool> ()->default_value (false)->implicit_value (true), "show both original and decoded PointClouds graphically")
  ("point_resolution,p", po::value<double> ()->default_value (0.20), "XYZ resolution of point coordinates")
  ("octree_resolution,r", po::value<double> ()->default_value (0.20), "voxel size")

  // V2 specific
  ("octree_bits,b", po::value<int> ()->default_value (11), "octree resolution (bits)")
  ("color_bits,c", po::value<int> ()->default_value (8), "color resolution (bits)")
  ("enh_bits,e", po::value<int> ()->default_value (0), "bits to code the points towards the center ")
  ("color_coding_type,t", po::value<int> ()->default_value (1), "pcl=0,jpeg=1 or graph transform ")
  ("macroblock_size,m",po::value<int> ()->default_value (16), "size of macroblocks used for predictive frame (must be power of 2)")
  ("keep_centroid", po::value<int> ()->default_value (0), "keep voxel grid positions or not ")
  ("create_scalable", po::value<bool> ()->default_value (false), "create scalable bitstream (not yet implemented)")
  ("do_connectivity_coding", po::value<bool> ()->default_value (false), "connectivity coding (not yet implemented)")
  ("icp_on_original", po::value<bool> ()->default_value (false),"icp_on_original ") // iterative closest point
  ("jpeg_quality,j", po::value<int> ()->default_value (0), "jpeg quality parameter ")
  ("do_delta_coding,d", po::value<bool> ()->default_value (false),"use delta (predictive) en(de)coding ")
  ("do_quality_computation,q", po::value<bool> ()->default_value (false),"compute quality of en(de)coding")
  ("do_icp_color_offset",po::value<bool> ()->default_value (false), "do color offset coding on predictive frames")
  ("num_threads,n",po::value<int> ()->default_value (1), "number of omp cores (1=default, 1 thread, no parallel execution)")
  ("intra_frame_quality_csv", po::value<string>()->default_value("intra_frame_quality.csv")," intra frame coding quality results file name (.csv file)")
  ("predictive_quality_csv",po::value<string>()->default_value("predictive_quality.csv"), " predictive coding quality results file name (.csv file)")
  ("debug_level",po::value<int> ()->default_value (0), "debug print level (0=no debug print, 3=all debug print)")
    ;
  pod_.add ("input_directories", -1);
 }

inline void
print_options (po::variables_map vm) {
  for (po::variables_map::iterator it = vm.begin (); it != vm.end (); it++) {
    cout << "\t " << it->first;
    if ( ( (boost::any)it->second.value ()).empty ()) {
      cout << " (empty)";
    }
    if (vm[it->first].defaulted () || it->second.defaulted ()) {
      cout << " (default)";
    }
    cout << "=";
    bool is_char;
    try {
      boost::any_cast<const char *> (it->second.value ());
      is_char = true;
    } catch (const boost::bad_any_cast &) {
      is_char = false;
    }
    bool is_str;
    try {
      boost::any_cast<std::string> (it->second.value ());
      is_str = true;
    } catch (const boost::bad_any_cast &) {
      is_str = false;
    }
    bool is_vector;
    try {
      boost::any_cast<vector<int> > (it->second.value ());
      is_vector = true;
    } catch (const boost::bad_any_cast &) {
      is_vector = false;
    }
    
    if ( ( (boost::any)it->second.value ()).type () == typeid (int)) {
      std::cout << vm[it->first].as<int> () << std::endl;
    } else if ( ( (boost::any)it->second.value ()).type () == typeid (bool)) {
      std::cout << vm[it->first].as<bool> () << std::endl;
    } else if ( ( (boost::any)it->second.value ()).type () == typeid (double)) {
      std::cout << vm[it->first].as<double> () << std::endl;
    } else if (is_char) {
      std::cout << vm[it->first].as<const char * > () << std::endl;
    } else if (is_str) {
      std::string temp = vm[it->first].as<std::string> ();
      if (temp.size ()) {
        std::cout << temp << std::endl;
      } else {
        std::cout << "<empty>" << std::endl;
      }
    } else if (is_vector) {
      std::vector<int> temp = vm[it->first].as<std::vector<int> > ();
      if (temp.size ()) {
        cout << "{";
        for (int i = 0; i < temp.size (); i++) {
          if (i) cout << ",";
          cout << temp[i];
        }
        cout << "}\n";
      } else {
        std::cout << "<empty>" << std::endl;
      }
    } else { // Assumes that the only left is vector<string>
      try {
        std::vector<std::string> vect = vm[it->first].as<std::vector<std::string> > ();
        unsigned int i = 0;
        for (std::vector<std::string>::iterator oit=vect.begin ();
             oit != vect.end (); oit++, ++i) {
          std::cout << "\r> " << it->first << "[" << i << "]=" << (*oit) << std::endl;
        }
      } catch (const boost::bad_any_cast &) {
        std::cout << "UnknownType (" << ( (boost::any)it->second.value ()).type ().name () << ")" << std::endl;
      }
    }
  }
}

template<typename PointT>
bool
evaluate_comp_impl<PointT>::get_options (int argc, char** argv)
{
// first parse configuration file, then parse command line options
//  po::variables_map vm;
//  po::store (po::parse_config_file (in_conf, desc), vm);
//  po::notify (vm);
  // Check if optional file 'parameter_config.txt' is present in parent or current working directory
  bool use_parent = true, has_config = true, return_value = true;
  std::ifstream config_file ("..//parameter_config.txt");
  if (config_file.fail ()) {
    use_parent = false;
    config_file.open ("parameter_config.txt");
    if (config_file.fail ()) {
      std::cerr << " Optional file 'parameter_config.txt' not found in '" << boost::filesystem::current_path ().string ().c_str () << "' or its parent.\n";
      has_config = false;
    }
  }
  if (has_config) {
    if (debug_level_ >= 2) {
      std::cerr << "Using '" <<  boost::filesystem::current_path ().c_str ();
      if (use_parent) {
        std::cerr << "/..";
      }
      std::cerr << "/parameter_config.txt'\n";
    }
  }
  // first parse command line options, then parse configuration file
  // Since parsed options are immutable, this has the effect that command line options take precedence
  bool allow_unregistered = true;
  po::parsed_options parsed_options = po::command_line_parser (argc, argv).options (desc_).positional (pod_).allow_unregistered ().run ();
  std::vector<std::string> unrecognized_options = po::collect_unrecognized (parsed_options.options, po::exclude_positional);
  if (unrecognized_options.size () > 0) {
    cerr << "Unrecognized options on command line:\n";
    for (std::vector<std::string>::iterator itr= unrecognized_options.begin (); itr != unrecognized_options.end (); itr++) {
      std::string unrecognized = *itr;
      std::cerr << unrecognized.c_str () << "\n";
    }
  } else {
    po::store (parsed_options, vm_);
    po::notify (vm_);
    parsed_options = po::parse_config_file (config_file, desc_, allow_unregistered);
    unrecognized_options = po::collect_unrecognized (parsed_options.options, po::exclude_positional);
    if (unrecognized_options.size () > 0) {
      cerr << "Unrecognized options in configuration file:\n";
      for (std::vector<std::string>::iterator itr= unrecognized_options.begin (); itr != unrecognized_options.end (); itr++) {
        std::string unrecognized = *itr;
        std::cerr << unrecognized.c_str () << "\n";
      }
      return_value = false;
    } else {
      po::store (parsed_options, vm_);
      po::notify (vm_);
    }
  }
  return return_value;
}

template<typename PointT>
void
evaluate_comp_impl<PointT>::assign_option_values ()
{
  algorithm_ = vm_["algorithm"].template as<std::string> ();
  group_size_ = vm_["group_size"].template as<int> ();
  K_outlier_filter_ = vm_["K_outlier_filter"].template as<int> ();
  radius_ = vm_["radius"].template as<double> ();
  bb_expand_factor_ = vm_["bb_expand_factor"].template as<double> ();
  algorithm_ = vm_["algorithm"].template as<std::string> ();
  show_statistics_ = vm_["show_statistics"].template as<bool> ();
  enh_bits_ = vm_["enh_bits"].template as<int> ();
  octree_bits_ = vm_["octree_bits"].template as<int> ();
  color_bits_ = vm_["color_bits"].template as<int> ();
  visualization_ = vm_["visualization"].template as<bool> ();
  if (vm_.count ("input_directories")) {
    input_directories_ = vm_["input_directories"].template as<std::vector<std::string> > ();
  }
  output_directory_ = vm_["output_directory"].template as<std::string> ();
  if (algorithm_ == "V2")
  {
    color_coding_type_ = vm_["color_coding_type"].template as<int> ();
    macroblock_size_ = vm_["macroblock_size"].template as<int> ();
    keep_centroid_ = vm_["keep_centroid"].template as<int> ();
    create_scalable_ = vm_["create_scalable"].template as<bool> ();
    jpeg_quality_ = vm_["jpeg_quality"].template as<int> ();
    do_delta_coding_ = vm_["do_delta_coding"].template as<bool> ();
    do_quality_computation_ = vm_["do_quality_computation"].template as<bool> ();
    icp_on_original_ = vm_["icp_on_original"].template as<bool> ();
    do_icp_color_offset_ = vm_["do_icp_color_offset"].template as<bool> ();
    num_threads_ = vm_["num_threads"].template as<int> ();
    intra_frame_quality_csv_ = vm_["intra_frame_quality_csv"].template as<string>();
    predictive_quality_csv_ = vm_["predictive_quality_csv"].template as<string>();
  }
}
template<typename PointT>
void
evaluate_comp_impl<PointT>::complete_initialization ()
{
  // encoder, decoder
  if (algorithm_ == "V1")
  {
    encoder_V1_ = boost::shared_ptr<pcl::io::OctreePointCloudCompression<PointT> > (
                    new pcl::io::OctreePointCloudCompression<PointT> (
                      pcl::io::MANUAL_CONFIGURATION,
                      show_statistics_,
                      octree_bits_ > 0 ? std::pow ( 2.0, -1.0 * octree_bits_) : point_resolution_,
                      octree_bits_ > 0 ? std::pow ( 2.0, -1.0 * octree_bits_) : point_resolution_,
                      false, // no intra voxel coding in this first version of the codec
                      0, // i_frame_rate,
                      color_bits_ > 0 ? true : false,
                      color_bits_
                    )
                  );
	decoder_V1_ = boost::shared_ptr<pcl::io::OctreePointCloudCompression<PointT> > (
                    new pcl::io::OctreePointCloudCompression<PointT> (
                      pcl::io::MANUAL_CONFIGURATION,
                      false,
                      octree_bits_ > 0 ? std::pow ( 2.0, -1.0 * octree_bits_) : point_resolution_,
                      octree_bits_ > 0 ? std::pow ( 2.0, -1.0 * octree_bits_) : point_resolution_,
                      false, // no intra voxel coding in this first version of the codec
                      0, // i_frame_rate,
                      color_bits_ > 0 ? true : false,
                      color_bits_
                    )
                  );
  }
  else
  {
    if (algorithm_ == "V2")
    {
      encoder_V2_ = boost::shared_ptr<pcl::io::OctreePointCloudCodecV2<PointT> > (
                           new pcl::io::OctreePointCloudCodecV2<PointT> (
                             pcl::io::MANUAL_CONFIGURATION,
                             show_statistics_,
                             octree_bits_ > 0 ? std::pow ( 2.0, -1.0 * (octree_bits_ + enh_bits_) ) : // XXX
                             point_resolution_,
                             octree_bits_ > 0 ? std::pow ( 2.0, -1.0 * octree_bits_) : // XXX
                             point_resolution_,
                             true, // no intra voxel coding in this first version of the codec
                             0, // i_frame_rate,
                             color_bits_ > 0 ? true : false,
                             color_bits_,
                             color_coding_type_,
                             keep_centroid_,
                             create_scalable_, // not implemented
                             false, // do_connectivity_coding_ not implemented
                             jpeg_quality_,
                             num_threads_
                    ));
	  decoder_V2_ = boost::shared_ptr<pcl::io::OctreePointCloudCodecV2<PointT> > (
                             new pcl::io::OctreePointCloudCodecV2<PointT> (
                             pcl::io::MANUAL_CONFIGURATION,
                             false,
                             octree_bits_ > 0 ? std::pow ( 2.0, -1.0 * (octree_bits_ + enh_bits_) ) : // XXX
                             point_resolution_,
                             octree_bits_ > 0 ? std::pow ( 2.0, -1.0 * octree_bits_) : // XXX
                             point_resolution_,
                             true, // no intra voxel coding in this first version of the codec
                             0, // i_frame_rate,
                             color_bits_ > 0 ? true : false,
                             color_bits_,
                             color_coding_type_,
                             keep_centroid_,
                             create_scalable_, // not implemented
                             false, // do_connectivity_coding_, not implemented
                             jpeg_quality_,
                             num_threads_
                      ));
      encoder_V2_->setMacroblockSize (macroblock_size_);
      // icp offset coding
      encoder_V2_->setDoICPColorOffset (do_icp_color_offset_);
    }
  }
  if (output_directory_ != "")
  {
    boost::filesystem::path out_dir_path(output_directory_);
    if ( ! exists(out_dir_path) && ! boost::filesystem::create_directory(out_dir_path))
    {
      std::cout << "Can't create output_directory '"+output_directory_+"'\n";;
    }
  }
}
    
template<typename PointT>
void
evaluate_comp_impl<PointT>::do_outlier_removal (std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >& group)
{
  pcl::io::OctreePointCloudCodecV2 <PointT>::remove_outliers (group, K_outlier_filter_, radius_, debug_level_);
}
      
template<typename PointT>
pcl::io::BoundingBox
evaluate_comp_impl<PointT>::do_bounding_box_normalization (std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >& group)
{
  vector<float> dyn_range, offset;
  vector<pcl::io::BoundingBox, Eigen::aligned_allocator<pcl::io::BoundingBox> >  bounding_boxes (group.size ());
  return pcl::io::OctreePointCloudCodecV2 <PointT>::normalize_pointclouds (group, bounding_boxes, bb_expand_factor_,  dyn_range, offset, debug_level_);
 }
                                  
template<typename PointT>
void
evaluate_comp_impl<PointT>::do_encoding (boost::shared_ptr<pcl::PointCloud<PointT> > pointcloud, std::stringstream* stream, QualityMetric & qualityMetric)
{
  pcl::console::TicToc tt;
  int initial_stream_pos = stream->tellp ();
  if (algorithm_ == "V1")
  {
    tt.tic ();
    encoder_V1_->encodePointCloud (pointcloud , *stream);
    qualityMetric.encoding_time_ms = tt.toc ();
  }
  else
  {
    if (algorithm_ == "V2")
    {
      tt.tic ();
      encoder_V2_->encodePointCloud (pointcloud , *stream);
      qualityMetric.encoding_time_ms = tt.toc ();
      // store the partial bytes sizes
      uint64_t *c_sizes = encoder_V2_->getPerformanceMetrics ();
      qualityMetric.byte_count_octree_layer = c_sizes[0];
      qualityMetric.byte_count_centroid_layer = c_sizes[1];
      qualityMetric.byte_count_color_layer = c_sizes[2];
    }
  }
  qualityMetric.compressed_size = (int) stream->tellp () - initial_stream_pos;
  cout << " octreeCoding " << qualityMetric.compressed_size << " bytes  base layer  " << endl;
}
    
template<typename PointT>
void
evaluate_comp_impl<PointT>::do_decoding (std::stringstream* coded_stream, boost::shared_ptr<pcl::PointCloud<PointT> > pointcloud, QualityMetric & qualityMetric)
{
  pcl::console::TicToc tt;
  tt.tic ();
  if (algorithm_ == "V1")
  {
    decoder_V1_->decodePointCloud (*coded_stream, pointcloud);
  }
  else
  {
    if (algorithm_ == "V2")
    {
      decoder_V2_->decodePointCloud (*coded_stream, pointcloud);
    }
  }
  qualityMetric.decoding_time_ms = tt.toc ();
}

template<typename PointT>
void
evaluate_comp_impl<PointT>::do_delta_encoding (boost::shared_ptr<pcl::PointCloud<PointT> > i_cloud,
                                                     boost::shared_ptr<pcl::PointCloud<PointT> > p_cloud,
                                                     boost::shared_ptr<pcl::PointCloud<PointT> > out_cloud,
                                                     std::stringstream* i_stream,
                                                     std::stringstream* p_stream,
                                                     QualityMetric & qualityMetric)
{
  pcl::console::TicToc tt;
  tt.tic ();
  encoder_V2_->encodePointCloudDeltaFrame (i_cloud, p_cloud, out_cloud, *i_stream, *p_stream, icp_on_original_, false);
  qualityMetric.encoding_time_ms = tt.toc ();
  qualityMetric.byte_count_octree_layer = i_stream->tellp ();
  qualityMetric.byte_count_centroid_layer = p_stream->tellp ();
  qualityMetric.compressed_size = i_stream->tellp () + p_stream->tellp ();
  qualityMetric.byte_count_color_layer= 0;
}
    
template<typename PointT>
void
evaluate_comp_impl<PointT>::do_delta_decoding (std::stringstream* i_stream,
                                                     std::stringstream* p_stream,
                                                     boost::shared_ptr<pcl::PointCloud<PointT> > i_cloud,
                                                     boost::shared_ptr<pcl::PointCloud<PointT> > out_cloud,
                                                     QualityMetric & qualityMetric)
{
  pcl::console::TicToc tt;
  tt.tic ();
  encoder_V2_->decodePointCloudDeltaFrame (i_cloud, out_cloud, *i_stream, *p_stream);
  qualityMetric.decoding_time_ms = tt.toc ();
}
    
    
template<typename PointT>
void
evaluate_comp_impl<PointT>::do_quality_computation (boost::shared_ptr<pcl::PointCloud<PointT> > & reference_pointcloud,
                                                        boost::shared_ptr<pcl::PointCloud<PointT> > & pointcloud,
                                                        QualityMetric & quality_metric)
{
  // compute quality metric
  computeQualityMetric<PointT> (*reference_pointcloud, *pointcloud, quality_metric);
}
    
template<typename PointT>
void
evaluate_comp_impl<PointT>::do_output (std::string path, boost::shared_ptr<pcl::PointCloud<PointT> > pointcloud, QualityMetric & qualityMetric)
{
  // write the .ply file by converting to point cloud2 and then to polygon mesh
  pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2( *pointcloud, *cloud2);
  pcl::PLYWriter writer;
  writer.write(output_directory_+"/"+path, cloud2);
}

template<typename PointT>
void
evaluate_comp_impl<PointT>::do_visualization (std::string id, boost::shared_ptr<pcl::PointCloud<PointT> > pc)
{
  if ( ! visualization_) return;

#ifdef WITH_VTK
  static std::map <std::string, ViewerPtr> viewers;
  static int viewer_index_ = 0;
  static vector <vtkRect<int> > viewer_window_; // x,y,w,h
  ViewerPtr viewer = viewers[id];
  vtkObject::GlobalWarningDisplayOff();

  if ( ! viewer)
  {
    viewer = new pcl::visualization::PCLVisualizer (id);
    viewers[id] = viewer;
    vtkRenderWindow* vrwp = viewer->getRenderWindow ();
    vtkRect<int> this_window;
    // TBD Find better way for window positioning, this setting works only for Portrait Oriented Screen
    if (viewer_index_ == 0)
    {
      int* window_size_p = vrwp->GetSize ();
      this_window = vtkRect<int> (0 /*window_position_p[0]*/, 800/*ss[1] - window_size_p[1] window_position_p[1]*/, window_size_p[0], window_size_p[1]);
    }
    else
    {
      this_window = vtkRect<int> (viewer_window_[viewer_index_ -1].GetX()+viewer_window_[0].GetWidth()*(viewer_index_ % 2),
                                  viewer_window_[viewer_index_ -1].GetY()-viewer_window_[0].GetHeight()*(viewer_index_ / 2),
                                  viewer_window_[viewer_index_ -1].GetWidth(),
                                  viewer_window_[viewer_index_ -1].GetHeight());
    }
    viewer_window_.push_back (this_window);
    viewer->setBackgroundColor (.773, .78, .769); // RAL 7035 light grey
    // we use the internal versions of these SetXX methods to avoid unwanted side-effects
    vrwp->SetPosition (this_window.GetX(), this_window.GetY());
    vrwp->SetSize (this_window.GetWidth(), this_window.GetHeight());
    viewer_index_++;
  }
  if (pc == NULL)
  {
    delete(viewer);
  }
  else
  {  // show the PointCloud in its viewer
    viewer->removePointCloud ();
    viewer->addPointCloud<PointT> (pc);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    viewer->spinOnce(1);
  }
#else //WITH_VTK
  static bool warning_given = false;
  if ( ! warning_given)
  {
    PCL_WARN("No visualization configured");
    warning_given = true;
  }
#endif//WITH_VTK
}

// aux. functions for file reading
using namespace boost::filesystem;
using namespace pcl;
template<typename PointT>
int
load_pcd_file (std::string path, boost::shared_ptr<pcl::PointCloud<PointT> > pc)
{
  int rv = 1;
  PCDReader pcd_reader;
  if (pcd_reader.read (path, *pc) <= 0)
  {
    rv = 0;
  }
  return rv;
}

template<typename PointT>
int
load_ply_file (std::string path, boost::shared_ptr<pcl::PointCloud<PointT> > pc)
{
  int rv = 1;
  PLYReader ply_reader;
/* next straighforward code crashes, work around via PolygonMesh *
   PCLPointCloud2 pc2;
   if (rv && ply_reader.read (path, pc2) < 0)
   {
     fromPCLPointCloud2 (pc2, *pc);
     rv = 0;
   }
*/
  PolygonMesh mesh;
  if (rv && ply_reader.read (path, mesh) >= 0) {
    pcl::fromPCLPointCloud2 (mesh.cloud, *pc);
  } else {
    rv= 0;
  }
  return rv;
}

int
get_filenames_from_dir (std::string directory, vector<std::string>& filenames)
{
    int rv = 0;
    const char* dir_name = directory.c_str();
    if ( ! boost::filesystem::is_directory (dir_name)) {
        std::cerr << "'" << directory << "' is not a directory.\n";
        return -1;
    }
    // order of filenames returned by directory_iterator is undefined
    namespace fs = boost::filesystem;
    //#pragma omp parallel for
    for (fs::directory_iterator itr (directory); itr != fs::directory_iterator (); itr++)
    {
        fs::directory_entry de = *itr;
        filenames.push_back (de.path ().string ());
    }
    std::sort (filenames.begin (), filenames.end ());
    return rv;
}

template<typename PointT>
bool
load_input_cloud (std::string filename, boost::shared_ptr<pcl::PointCloud<PointT> > &cloud)
{
  bool rv = false;
  if (boost::ends_with (filename, ".ply"))
  {
    if (load_ply_file (filename, cloud))
    {
      rv = true;
    }
  }
  else
  {
    if (boost::ends_with (filename, ".pcd"))
    {
      if (load_pcd_file (filename, cloud))
      {
        rv = true;
      }
    }
  }
  return rv;
}

template<typename PointT>
bool
evaluate_comp_impl<PointT>::evaluate ()
{
  bool return_value = true;
  
  try
  {
    initialize_options_description ();
    if ( ! get_options (argc_, argv_))
    {
      return false;
    }
    debug_level_ = vm_["debug_level"].template as<int> ();
    if (debug_level_ > 0)
    {
      std::cout << "debug_level=" << debug_level_ << "\n";
      print_options (vm_);
    }
#ifdef WITH_VTK
    std::cout << "WITH_VTK='" << WITH_VTK << "'\n";
#endif/*WITH_VTK*/
    if (vm_.count ("help"))
    {
      std::cout << desc_ << "\n";
      return return_value;
    }
    assign_option_values ();
    complete_initialization ();
    if (input_directories_.size() > 1)
    {
      cout << "Fusing multiple directories not implemented.\n";
      return (false);
    }
    if (input_directories_.size() < 1)
    {
      cout << "Need to specify a directory containing Point Cloud files (.pcd or .ply).\n";
      return (false);
    }
    std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >  group, encoder_output_clouds;
    int count = 0;
    std::ofstream intra_frame_quality_csv;
    stringstream compression_settings;
    compression_settings << "octree_bits=" << octree_bits_ << " color_bits=" <<  color_bits_ << " enh._bits=" << enh_bits_ << "_colortype=" << color_coding_type_ << " centroid=" << keep_centroid_;

    if (intra_frame_quality_csv_ != "")
    {
      intra_frame_quality_csv.open(intra_frame_quality_csv_.c_str());
      QualityMetric::print_csv_header(intra_frame_quality_csv);
    }
    std::ofstream predictive_quality_csv;
    if (predictive_quality_csv_ != "")
    {
      predictive_quality_csv.open(predictive_quality_csv_.c_str());
      QualityMetric::print_csv_header(predictive_quality_csv);
    }
    vector<std::string> filenames;
    if (get_filenames_from_dir (*input_directories_.begin(), filenames) != 0) return false;
    for (std::vector<std::string>::iterator itr = filenames.begin (); itr != filenames.end (); itr++)
    {
      std::string filename = *itr;
      if (output_index_ == -1) { // get index of first file
        std::stringstream ss(filename);
        string tmp;
        ss >> tmp >> output_index_;
        if (output_index_ == -1) // no index found
        output_index_ = 0;
      }
      boost::shared_ptr<pcl::PointCloud<PointT> > pc (new PointCloud<PointT> ());
      if ( ! load_input_cloud(filename, pc))
      {
        continue;
      }
      group.push_back(pc->makeShared());
      count++;
      if (group_size_ == 0 && count < filenames.size ())
      {
        continue;
      }
      // encode the group for each set of 'group_size' filenames, and the final set
      if (group_size_ == 0 || count == filenames.size () || count % group_size_ == 0)
      {
        evaluate_group (group, compression_settings, intra_frame_quality_csv, predictive_quality_csv);
        complete_initialization();
        // start new group
        group.clear ();
        count = 0;
      }
    }
  } catch (boost::exception &e) {
    std::cerr << boost::diagnostic_information (e) << "\n";
    return_value = false;
  }
  if (visualization_)
  { // remove data structures related to visualzation
    do_visualization ("Original", boost::shared_ptr<PointCloud<PointT> >());
    do_visualization ("Decoded",  boost::shared_ptr<PointCloud<PointT> >());
    do_visualization ("Delta Decoded",  boost::shared_ptr<PointCloud<PointT> >());
  }
  return return_value;
}
template<typename PointT>
bool
evaluate_comp_impl<PointT>::evaluator (encoder_params param, boost::shared_ptr<pcl::PointCloud<PointT> > pointcloud, std::ostream& comp_frame)
{
  bool return_value = true;
  
  try
  {
    //initialize_options_description ();
    //if ( ! get_options (argc_, argv_))
    //{
      //return false;
    //}
    debug_level_ = vm_["debug_level"].template as<int> ();
    if (debug_level_ > 0)
    {
      std::cout << "debug_level=" << debug_level_ << "\n";
      //print_options (vm_);
    }
#ifdef WITH_VTK
    std::cout << "WITH_VTK='" << WITH_VTK << "'\n";
#endif/*WITH_VTK*/
    if (vm_.count ("help"))
    {
      std::cout << desc_ << "\n";
      return return_value;
    }
    assign_option_values ();
    complete_initialization ();
    if (input_directories_.size() > 1)
    {
      cout << "Fusing multiple directories not implemented.\n";
      return (false);
    }
    if (input_directories_.size() < 1)
    {
      cout << "Need to specify a directory containing Point Cloud files (.pcd or .ply).\n";
      return (false);
    }
    std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >  group, encoder_output_clouds;
    int count = 0;
    std::ofstream intra_frame_quality_csv;
    stringstream compression_settings;
    compression_settings << "octree_bits=" << octree_bits_ << " color_bits=" <<  color_bits_ << " enh._bits=" << enh_bits_ << "_colortype=" << color_coding_type_ << " centroid=" << keep_centroid_;

    if (intra_frame_quality_csv_ != "")
    {
      intra_frame_quality_csv.open(intra_frame_quality_csv_.c_str());
      QualityMetric::print_csv_header(intra_frame_quality_csv);
    }
    std::ofstream predictive_quality_csv;
    if (predictive_quality_csv_ != "")
    {
      predictive_quality_csv.open(predictive_quality_csv_.c_str());
      QualityMetric::print_csv_header(predictive_quality_csv);
    }
    vector<std::string> filenames;
    if (get_filenames_from_dir (*input_directories_.begin(), filenames) != 0) return false;
    for (std::vector<std::string>::iterator itr = filenames.begin (); itr != filenames.end (); itr++)
    {
      std::string filename = *itr;
      if (output_index_ == -1) { // get index of first file
        std::stringstream ss(filename);
        string tmp;
        ss >> tmp >> output_index_;
        if (output_index_ == -1) // no index found
        output_index_ = 0;
      }
      boost::shared_ptr<pcl::PointCloud<PointT> > pc (new PointCloud<PointT> ());
      if ( ! load_input_cloud(filename, pc))
      {
        continue;
      }
      group.push_back(pc->makeShared());
      count++;
      if (group_size_ == 0 && count < filenames.size ())
      {
        continue;
      }
      // encode the group for each set of 'group_size' filenames, and the final set
      if (group_size_ == 0 || count == filenames.size () || count % group_size_ == 0)
      {
        evaluate_group (group, compression_settings, intra_frame_quality_csv, predictive_quality_csv);
        complete_initialization();
        // start new group
        group.clear ();
        count = 0;
      }
    }
  } catch (boost::exception &e) {
    std::cerr << boost::diagnostic_information (e) << "\n";
    return_value = false;
  }
  if (visualization_)
  { // remove data structures related to visualzation
    do_visualization ("Original", boost::shared_ptr<PointCloud<PointT> >());
    do_visualization ("Decoded",  boost::shared_ptr<PointCloud<PointT> >());
    do_visualization ("Delta Decoded",  boost::shared_ptr<PointCloud<PointT> >());
  }
  return return_value;
}


template<typename PointT>
bool
evaluate_comp_impl<PointT>::evaluate_group(std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >& group,
            stringstream& compression_settings, std::ofstream& intra_frame_quality_csv, std::ofstream& predictive_quality_csv)
{
  bool rv = true; // return value
  // create a deep copy of the group, as the pointclouds may be modified and the original is needed to compare results
  std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > > working_group;
  for (typename std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >::iterator itr = group.begin (); itr != group.end (); itr++)
  {
    boost::shared_ptr<pcl::PointCloud<PointT> > point_cloud = *itr;
    working_group.push_back (point_cloud->makeShared ());
  }
  if (K_outlier_filter_ > 0) do_outlier_removal (working_group);
  pcl::io::BoundingBox bb; // bounding box of this working_group
  if (bb_expand_factor_ > 0.0) bb = do_bounding_box_normalization (working_group);
  int working_group_size = working_group.size();
  vector<float> icp_convergence_percentage (working_group_size);
  vector<float> shared_macroblock_percentages (working_group_size);
//  boost::shared_ptr<pcl::PointCloud<PointT> > dpc; // decoded point cloud
  for (int i = 0; i < working_group.size (); i++)
  {
    boost::shared_ptr<pcl::PointCloud<PointT> > pc = working_group[i];
    boost::shared_ptr<pcl::PointCloud<PointT> > original_pc = group[i]->makeShared ();
    stringstream ss;
    QualityMetric achieved_quality;
    int i_strm_pos_cur = 0, i_strm_pos_prev = 0; // current and previous position in i-code stream
    int p_strm_pos_cur = 0, p_strm_pos_prev = 0; // current and previous position in p-code stream
      
    // encode pointcloud to string stream
    do_encoding (pc, &ss, achieved_quality);
    // decode the string stream
    string s = ss.str ();
    std::stringstream coded_stream (s);//ss.str ());
    int group_size = group.size ();
    boost::shared_ptr<pcl::PointCloud<PointT> > output_pointcloud (new pcl::PointCloud<PointT> ()), opc (new pcl::PointCloud<PointT> ());
    opc = group[i];
    do_decoding (&coded_stream, output_pointcloud, achieved_quality);
    if (do_quality_computation_)
    {
      do_quality_computation (pc, output_pointcloud, achieved_quality);
      if (intra_frame_quality_csv_ != "")
      {
        achieved_quality.print_csv_line(compression_settings.str(), intra_frame_quality_csv);
      }
    }
    if (output_directory_ != "")
    {
      do_output ( "pointcloud_" + boost::lexical_cast<string> (++output_index_) + ".ply", output_pointcloud, achieved_quality);
    }
    if (bb_expand_factor_ > 0.0) pcl::io::OctreePointCloudCodecV2 <PointT>::restore_scaling (output_pointcloud, bb);
    // Note that the quality of the en/decompression was computed on the transformed (bb aligned) pointclouds
    do_visualization ("Original", opc);
    do_visualization ("Decoded", output_pointcloud);
    // test and evaluation iterative closest point predictive coding
    if (algorithm_ == "V2" && do_delta_coding_ && bb_expand_factor_ >= 0  // bounding boxes were aligned
        && i > 0 && i+1 < group_size)
    {
      boost::shared_ptr<pcl::PointCloud<PointT> > predicted_pc (new pcl::PointCloud<PointT> ());
      cout << " delta coding frame nr " << i+1 << endl;
      stringstream p_frame_pdat, p_frame_idat;
      QualityMetric predictive_quality;
                  
      do_delta_encoding (icp_on_original_ ? pc : encoder_V2_->getOutputCloud (), working_group[i+1], predicted_pc, &p_frame_idat, &p_frame_pdat, predictive_quality);
      // quality
      shared_macroblock_percentages[i] = encoder_V2_->getMacroBlockPercentage ();
      icp_convergence_percentage[i] =  encoder_V2_->getMacroBlockConvergencePercentage ();
                  
      p_strm_pos_prev = p_strm_pos_cur;
      p_strm_pos_cur = p_frame_pdat.tellp ();
      i_strm_pos_prev = i_strm_pos_cur;
      i_strm_pos_cur = p_frame_idat.tellp ();
      cout << " encoded a predictive frame: coded " << (i_strm_pos_cur - i_strm_pos_prev) << " bytes intra and " << (p_strm_pos_cur - p_strm_pos_prev) << " inter frame encoded " <<endl;
      // create a deep copy of original pointcloud, for comparison
      do_delta_decoding (&p_frame_idat, &p_frame_pdat, output_pointcloud, predicted_pc, predictive_quality);
//    compute the quality of the resulting predictive frame
      pcl::io::OctreePointCloudCodecV2 <PointT>::restore_scaling (predicted_pc, bb);
      if (output_directory_ != "")
      {
        do_output ("delta_decoded_pc_" + boost::lexical_cast<string> (output_index_) + ".ply", predicted_pc, achieved_quality);
      }
      do_visualization ("Delta Decoded", predicted_pc);
      if (do_quality_computation_)
      {
        do_quality_computation (working_group[i+1], predicted_pc, predictive_quality);
        if (predictive_quality_csv_ != "")
        {
          predictive_quality.print_csv_line(compression_settings.str(), predictive_quality_csv);
        }
      }
    }
    output_pointcloud->clear (); // clear output cloud
  }
  return rv;
}
#endif /* evaluate_compression_hpp */
