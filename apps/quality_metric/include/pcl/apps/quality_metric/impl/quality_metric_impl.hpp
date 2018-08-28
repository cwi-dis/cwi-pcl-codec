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
//  quality_metric.hpp
//  quality_metric
//
//  Created by Kees Blom on 20/08/18.
//
//
#ifndef quality_metric_hpp
#define quality_metric_hpp

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

#ifdef WIN32
#include <direct.h>
#include <stdlib.h>
#include <stdio.h>
#endif//WIN32
#include <quality_metric.h>
#include <pcl/pcl_base.h>
#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>

template<typename PointT>
class quality_metric_impl : quality_metric
{
  // using boost::exception on errors
public:
  quality_metric_impl (int argc, char** argv) : quality_metric (argc, argv), debug_level_ (3) {};
  
  // options handling
  void initialize_options_description ();
  bool get_options (int argc, char** argv);
  bool assign_option_values ();bool evaluate (); // TBD need catch exceptions
  boost::shared_ptr<pcl::PointCloud<PointT> > get_pointcloud(std::string path);
  
private:
  // options processing
  po::options_description desc_;
  po::variables_map vm_;
  po::positional_options_description pod_;
  
  int debug_level_;
  QualityMethod quality_method_;
};

template<typename PointT>
void
quality_metric_impl<PointT>::initialize_options_description ()
{
  desc_.add_options ()
  ("help,h", "produce help message ")
  ("fileA,a", po::value<std::string> ()->default_value (""), "Path to original pointcloud file")
  ("fileB,b", po::value<std::string> ()->default_value (""), "Path to degenerated pointcloud file")
  ("method,m", po::value<std::string> ()->default_value ("NONE"), "Error computation method: NONE,SELECT,TCSVT,MAX_NN,NORMALISED,BT709")
  ("debug_level",po::value<int> ()->default_value (0), "debug print level (0=no debug print, 3=all debug print)")
  ;
}

inline void
print_options (po::variables_map vm) {
  for (po::variables_map::iterator it = vm.begin (); it != vm.end (); it++) {
    cerr << "\t " << it->first;
    if ( ( (boost::any)it->second.value ()).empty ()) {
      cerr << " (empty)";
    }
    if (vm[it->first].defaulted () || it->second.defaulted ()) {
      cerr << " (default)";
    }
    cerr << "=";
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
      cerr << vm[it->first].as<int> () << std::endl;
    } else if ( ( (boost::any)it->second.value ()).type () == typeid (bool)) {
      cerr << vm[it->first].as<bool> () << std::endl;
    } else if ( ( (boost::any)it->second.value ()).type () == typeid (double)) {
      cerr << vm[it->first].as<double> () << std::endl;
    } else if (is_char) {
      cerr << vm[it->first].as<const char * > () << std::endl;
    } else if (is_str) {
      std::string temp = vm[it->first].as<std::string> ();
      if (temp.size ()) {
        cerr << temp << std::endl;
      } else {
        cerr << "<empty>" << std::endl;
      }
    } else if (is_vector) {
      std::vector<int> temp = vm[it->first].as<std::vector<int> > ();
      if (temp.size ()) {
        cerr << "{";
        for (int i = 0; i < temp.size (); i++) {
          if (i) cerr << ",";
          cerr << temp[i];
        }
        cerr << "}\n";
      } else {
        cerr << "<empty>" << std::endl;
      }
    } else { // Assumes that the only left is vector<string>
      try {
        std::vector<std::string> vect = vm[it->first].as<std::vector<std::string> > ();
        unsigned int i = 0;
        for (std::vector<std::string>::iterator oit=vect.begin ();
             oit != vect.end (); oit++, ++i) {
          cerr << "\r> " << it->first << "[" << i << "]=" << (*oit) << std::endl;
        }
      } catch (const boost::bad_any_cast &) {
        cerr << "UnknownType (" << ( (boost::any)it->second.value ()).type ().name () << ")" << std::endl;
      }
    }
  }
}

template<typename PointT>
bool
quality_metric_impl<PointT>::get_options (int argc, char** argv)
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
      cerr << " Optional file 'parameter_config.txt' not found in '" << boost::filesystem::current_path ().string ().c_str () << "' or its parent.\n";
      has_config = false;
    }
  }
  if (has_config) {
    if (debug_level_ >= 2) {
      cerr << "Using '" <<  boost::filesystem::current_path ().c_str ();
      if (use_parent) {
        cerr << "/..";
      }
      cerr << "/parameter_config.txt'\n";
    }
  }
  // first parse command line options, then parse configuration file
  // Since parsed options are immutable, this has the effect that command line options take precedence
  bool allow_unregistered = true;
  po::parsed_options parsed_options = po::command_line_parser (argc, argv).options (desc_).positional (pod_).allow_unregistered ().run ();
  std::vector<std::string> unrecognized_options = po::collect_unrecognized (parsed_options.options, po::exclude_positional);
  if (unrecognized_options.size () > 0) {
    cerr << "Unrecognized options on command line:\n";
    for (std::vector<std::string>::iterator itr = unrecognized_options.begin (); itr != unrecognized_options.end (); itr++) {
      std::string unrecognized = *itr;
      cerr << unrecognized.c_str () << "\n";
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
        cerr << unrecognized.c_str () << "\n";
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
boost::shared_ptr<pcl::PointCloud<PointT> >
quality_metric_impl<PointT>::get_pointcloud(std::string path)
{
  boost::shared_ptr<pcl::PointCloud<PointT> > rv (new pcl::PointCloud<PointT> ());
  
  if (boost::ends_with (path, ".pcd"))
  {
    pcl::PCDReader pcd_reader;
    if (pcd_reader.read (path, *rv) < 0)
      throw std::runtime_error("Cannot read"+path);
  } else
    if (boost::ends_with (path, ".ply"))
    {
      pcl::PLYReader ply_reader;
      /* next straighforward code crashes, work around via PolygonMesh *
       PCLPointCloud2 pc2;
       if (ply_reader.read (path, pc2) >= 0)
       {
       fromPCLPointCloud2 (pc2, *rv);
       }
       */
      pcl::PolygonMesh mesh;
      if (ply_reader.read (path, mesh) >= 0)
      {
        pcl::fromPCLPointCloud2 (mesh.cloud, *rv);
      }
      else
      {
        throw std::runtime_error("Cannot read"+path);
      }
    }
  return rv;
}

template<typename PointT>
bool
quality_metric_impl<PointT>::assign_option_values ()
{
  bool return_value = true;
  cloud_a_ = vm_["fileA"].template as<std::string> ();
  cloud_b_ = vm_["fileB"].template as<std::string> ();
  method_ = vm_["method"].template as<std::string> ();
  quality_method_ = QualityMetric::get_QualityMethod(method_);
  if (quality_method_ | BBALIGNED)
  {
    cerr << "method 'BBALIGNED' not defined (ignored)"  << "\n";
  }
  return return_value;
}

template<typename PointT>
bool
quality_metric_impl<PointT>::evaluate ()
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
      cerr << "debug_level=" << debug_level_ << "\n";
      print_options (vm_);
    }
    if (vm_.count ("help"))
    {
      cerr << desc_ << "\n";
      return return_value;
    }
    if ( ! assign_option_values ()) return false;
    
  } catch (boost::exception &e) {
    cerr << boost::diagnostic_information (e) << "\n";
    return_value = false;
  }
  boost::shared_ptr<pcl::PointCloud<PointT> > cloudA = get_pointcloud (cloud_a_);
  boost::shared_ptr<pcl::PointCloud<PointT> > cloudB = get_pointcloud (cloud_b_);
  QualityMetric quality_metric(quality_method_);
  quality_metric.computeQualityMetric<PointT> (cloudA, cloudB);
  
  return return_value;
}

#endif /* quality_metric_hpp */
