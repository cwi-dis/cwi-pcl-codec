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
#include <quality_metricCLI.h>
#include <pcl/pcl_base.h>
#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>

template<typename PointT> 
class QualityMetricCLI_impl : public QualityMetricCLI
{
public: 
  QualityMetricCLI_impl(int argc, char** argv) : 
    QualityMetricCLI (argc, argv), debug_level_ (3) {};
  
  // options handling
  void initializeOptionsDescription ();
  bool getOptions (int argc, char** argv);
  bool assignOptionValues ();bool evaluate (); // TBD need catch exceptions
  boost::shared_ptr<pcl::PointCloud<PointT> > getPointCloud(std::string path);
  
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
QualityMetricCLI_impl<PointT>::initializeOptionsDescription ()
{
  desc_.add_options ()
  ("help,h", "produce help message ")
  ("fileA,a", po::value<std::string> ()->default_value (""), "Path to original pointcloud file")
  ("fileB,b", po::value<std::string> ()->default_value (""), "Path to degenerated pointcloud file")
  ("method,m", po::value<std::string> ()->default_value ("NONE"), "Quality computation method: NONE,SELECT,TCSVT,MAX_NN,NORMALISED,BT709")
  ("debug_level", po::value<int> ()->default_value (0), "debug print level (0=no debug print, 3=all debug print)")
  ;
}

inline void
printOptions (po::variables_map vm) {
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
QualityMetricCLI_impl<PointT>::getOptions (int argc, char** argv)
{
  // parse command line options
  bool return_value = true; //, allow_unregistered = true;
  po::parsed_options parsed_options = po::command_line_parser (argc, argv).options (desc_).positional (pod_).allow_unregistered ().run ();
  std::vector<std::string> unrecognized_options = po::collect_unrecognized (parsed_options.options, po::exclude_positional);
  if (unrecognized_options.size () > 0) {
    cerr << "Unrecognized options on command line:\n";
    for (std::vector<std::string>::iterator itr = unrecognized_options.begin (); itr != unrecognized_options.end (); itr++) {
      std::string unrecognized = *itr;
      cerr << unrecognized.c_str () << "\n";
    }
    return_value = false;
  }
  po::store (parsed_options, vm_);
  po::notify (vm_);

  return return_value;
}

template<typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT> >
QualityMetricCLI_impl<PointT>::getPointCloud(std::string path)
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
QualityMetricCLI_impl<PointT>::assignOptionValues ()
{
  bool return_value = true;
  cloud_a_ = vm_["fileA"].template as<std::string> ();
  cloud_b_ = vm_["fileB"].template as<std::string> ();
  method_ = vm_["method"].template as<std::string> ();
  quality_method_ = QualityMetricCLI::getQualityMethodFromString(method_);
  if (quality_method_ & BBALIGNED)
  {
    cerr << "method 'BBALIGNED' not defined (ignored)"  << "\n";
  }
  return return_value;
}

template<typename PointT>
bool
QualityMetricCLI_impl<PointT>::evaluate ()
{
  bool return_value = true;
  
  try
  {
    initializeOptionsDescription ();
  
    if ( ! getOptions (argc_, argv_))
    {
      return false;
    }
    if (argc_ <= 1 || vm_.count ("help"))
    {
      cerr << "Usage:\n" << desc_ << "\n";
      return return_value;
    }
    debug_level_ = vm_["debug_level"].template as<int> ();
    if (debug_level_ > 0)
    {
      cerr << "debug_level=" << debug_level_ << "\n";
      printOptions (vm_);
    }
    if (vm_.count ("help"))
    {
      cerr << desc_ << "\n";
      return return_value;
    }
    if ( ! assignOptionValues ()) return false;
    
  } catch (boost::exception &e) {
    cerr << boost::diagnostic_information (e) << "\n";
    return_value = false;
  }
  boost::shared_ptr<pcl::PointCloud<PointT> > cloudA = getPointCloud (cloud_a_);
  boost::shared_ptr<pcl::PointCloud<PointT> > cloudB = getPointCloud (cloud_b_);
  QualityMetric quality_metric(quality_method_);
  quality_metric.computeQualityMetric<PointT> (cloudA, cloudB);
  
  return return_value;
}

#endif /* quality_metric_hpp */
