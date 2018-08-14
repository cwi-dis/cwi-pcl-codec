 /*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
 *  Copyright (c) 2014, Centrum Wiskunde Informatica
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */
#ifndef POINT_CLOUD_QUALITY_HPP 
#define POINT_CLOUD_QUALITY_HPP

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <quality_metric.h>

//using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

/**
\brief helper function to convert RGB to YUV using ITU-R rec.601 (analog TV)
*/
template<typename PointT> void
convertRGBtoYUV_BT601(const PointT &in_rgb, float * out_yuv)
{
  // color space conversion to YUV on a 0-1 scale
  out_yuv[0] = float( ( 0.299 * in_rgb.r + 0.587 * in_rgb.g + 0.114 * in_rgb.b)/255.0 );
  out_yuv[1] = float( (-0.147 * in_rgb.r - 0.289 * in_rgb.g + 0.436 * in_rgb.b)/255.0 );
  out_yuv[2] = float( ( 0.615 * in_rgb.r - 0.515 * in_rgb.g - 0.100 * in_rgb.b)/255.0 );
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief helper function to convert RGB to YUV using ITU-R rec.709 (HDTV)
 */
template<typename PointT> void
convertRGBtoYUV_BT709(const PointT &in_rgb, float *out_yuv)
{
    // color space conversion to YUV
    out_yuv[0] = float( ( 0.2126 * in_rgb.r + 0.7152 * in_rgb.g + 0.0722 * in_rgb.b) / 255.0 );
    out_yuv[1] = float( (-0.1146 * in_rgb.r - 0.3854 * in_rgb.g + 0.5000 * in_rgb.b) / 255.0 + 0.5000 );
    out_yuv[2] = float( ( 0.5000 * in_rgb.r - 0.4542 * in_rgb.g - 0.0458 * in_rgb.b) / 255.0 + 0.5000 );
}
/**
 \brief helper function to convert RGB to YUV using one of implemented conversion schemes
 */
template<typename PointT> void
QualityMetric::convertRGBtoYUV(const PointT &in_rgb, float *out_yuv)
{
  if (quality_method_ & BT709)
    convertRGBtoYUV_BT709(in_rgb, out_yuv);
  else
    convertRGBtoYUV_BT601(in_rgb, out_yuv);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * function to compute normals on a pointclouds
 * @param pc the pointcloud
 * @param normals the pc containing the resulting normals
 * @param k number of points to to define normal plane
 * \note PointT typename of point used in point cloud
 * \author Kees Blom (Kees.Blom@cwi.nl) modified in accordance with mpeg-3dgc pc_error code
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
computeNormals(boost::shared_ptr<PointCloud<PointT> > pc, PointCloud<pcl::Normal> &normals, int k=3)
{
    print_info(stderr, "TBD: Computing Normals\n");
/**
  pcl::NormalEstimationOMP<PointT, pcl::Normal> normalEstimation(1024);
  boost::shared_ptr<pcl::search::KdTree<PointT> > tree (new pcl::search::KdTree<PointT>);

  normalEstimation.setInputCloud (pc);
  normalEstimation.setSearchMethod (tree);
  normalEstimation.setKSearch (k);
  normalEstimation.compute (normals);
 **/
}
/**
 \brief helper function to compute a homogen transformation matrix 'tm'
 * can be used to tramsform a pointcloud 'pc' s.t. the x,y,z coordinates of all points
 * will be within the bounding box defined by the points 'min_box' and 'max_box'
 */
template<typename PointT>
void
fitPointCloudInBox(boost::shared_ptr<PointCloud<PointT> >  pc, Eigen::Matrix4f* tm, PointT* min_box, PointT* max_box,
                   bool Procrustes)
{
  PointT p_max;
  PointT p_min;
  PointT delta;
  const pcl::PointCloud<PointT> cpc(*pc);
    
  // calculate the bounding box
  pcl::getMinMax3D<PointT> (cpc, p_min, p_max);
  delta.x = (max_box->x - min_box->x)/(p_max.x-p_min.x);
  delta.y = (max_box->y - min_box->y)/(p_max.y-p_min.y);
  delta.z = (max_box->z - min_box->z)/(p_max.z-p_min.z);
  if  ( ! Procrustes)
  {
    // for isotropic transformation, select the min in x,y,z
    float sf = std::min (std::min (delta.x, delta.y), delta.z);
    delta.x = delta.y = delta.z = sf;
  }
  // fill the transformation matrix
  Eigen::Matrix4f m;
  m << delta.x,   0,         0,          min_box->x - p_min.x,
  0,         delta.y,   0,               min_box->y - p_min.y,
  0,         0,         delta.z,         min_box->z - p_min.z,
  0,         0,         0,               1;
  // return the transformation matrix
  *tm = m;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * function to compute quality metric with bit settings
 * @param cloud_a original cloud
 * @param cloud_b decoded cloud
 * @param qual_metric return updated quality metric
 * \note PointT typename of point used in point cloud
 * \author Rufael Mekuria (rufael.mekuria@cwi.nl)
 * \author Kees Blom (Kees.Blom@cwi.nl) modified in accordance with mpeg-3dgc pc_error code
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void
QualityMetric::computeQualityMetric (boost::shared_ptr<PointCloud<PointT> > cloud_a, boost::shared_ptr<PointCloud<PointT> > cloud_b)
{
  print_highlight (stderr, "Computing Quality Metrics for Point Clouds !!\n");

  //! we log the time for computing the quality metric
  TicToc tt;
  tt.tic ();
    /* no normals in this version
  // In accordance with MPEG-3DGC PointCloud Compression 'pc_error' for D1 (point-2-point metric)
  // we use the largest nearest neighbour distance of all points in A for the peak signal value
  // For the D2 metric (point-to-plane) we implement the specification in
  // MPEG2017/N16763 Annex B
  PointCloud<pcl::Normal> normals_a;
  computeNormals(cloud_a, normals_a, 10);
  // convert normal vectors to regular ones
  result->resize(normals_a.size());
  for (int i=0; i<normals_a.size(); i++)
  {
    result->at(i).x =  normals_a.at(i).normal[0];
    result->at(i).y =  normals_a.at(i).normal[1];
    result->at(i).z =  normals_a.at(i).normal[2];
  }
     */
  pcl::search::KdTree<PointT> tree_a_self;
  tree_a_self.setInputCloud (cloud_a->makeShared ());
  float max_dist_a_self = -std::numeric_limits<float>::max ();

  // compare A to B
  pcl::search::KdTree<PointT> tree_b;
  tree_b.setInputCloud (cloud_b->makeShared ());
		
  // geometric differences A -> B
  float max_dist_a = -std::numeric_limits<float>::max ();
  double rms_dist_a = 0;
		
  // color differences A -> B
  double psnr_colors_yuv[3] = {0.0,0.0,0.0};
  double mse_colors_yuv[3]  = {0.0,0.0,0.0};

  //! compute the maximum and mean square distance between each point in a to the nearest point in b
  //! compute the mean square color error for each point in a to the nearest point in b
  for (size_t i = 0; i < cloud_a->points.size (); ++i)
  {
    std::vector<int> indices_self (2);
    std::vector<float> sqr_distances_self (2);
    std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);
    if (quality_method_ & MAX_NN)
    {
      // search the most nearby point in the cloud_a_self, other than cloud_a->points[i]
      tree_a_self.nearestKSearch (cloud_a->points[i], 2, indices_self, sqr_distances_self);
      if (sqr_distances_self[1] > max_dist_a_self)
        max_dist_a_self = sqr_distances_self[1];
    }
    // search the most nearby point in the cloud_b
    tree_b.nearestKSearch (cloud_a->points[i], 1, indices, sqr_distances);
    
    // haussdorf geometric distance (Linf norm)
    if (sqr_distances[0] > max_dist_a)
      max_dist_a = sqr_distances[0];
  
    // mean square geometric distance (L2 norm)
    rms_dist_a+=sqr_distances[0];

    ////////////////////////////////////////////////////////////////
	// compute quality metric for yuv colors
	// 1. convert to RGB to YUV
	// 2. compute accumulated square error colors a to b
	////////////////////////////////////////////////////////////////
	float out_yuv[3];
	float in_yuv[3];

    convertRGBtoYUV<PointT>(cloud_a->points[i],in_yuv);
	convertRGBtoYUV<PointT>(cloud_b->points[indices[0]],out_yuv);
		  
    mse_colors_yuv[0]+=((in_yuv[0] - out_yuv[0]) * (in_yuv[0] - out_yuv[0]));
    mse_colors_yuv[1]+=((in_yuv[1] - out_yuv[1]) * (in_yuv[1] - out_yuv[1]));
    mse_colors_yuv[2]+=((in_yuv[2] - out_yuv[2]) * (in_yuv[2] - out_yuv[2]));
  }
  // compare geometry of B to A (needed for symmetric metric)
  pcl::search::KdTree<PointT> tree_a;
  tree_a.setInputCloud (cloud_a->makeShared ());
  float max_dist_b = -std::numeric_limits<float>::max ();
  double rms_dist_b = 0;
  
  for (size_t i = 0; i < cloud_b->points.size (); ++i)
  {
    std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);
    tree_a.nearestKSearch (cloud_b->points[i], 1, indices, sqr_distances);
    if (sqr_distances[0] > max_dist_b)
      max_dist_b = sqr_distances[0];

    // mean square distance
    rms_dist_b+=sqr_distances[0];
  }
        
  ////////////////////////////////////////////////////////////////
  // calculate geometric error metrics
  // 1. compute left and right haussdorf
  // 2. compute left and right rms
  // 3. compute symmetric haussdorf and rms
  // 4. calculate psnr for symmetric rms error
  ////////////////////////////////////////////////////////////////

  max_dist_a = std::sqrt (max_dist_a);
  max_dist_b = std::sqrt (max_dist_b);

  rms_dist_a = std::sqrt (rms_dist_a/cloud_a->points.size ());
  rms_dist_b = std::sqrt (rms_dist_b/cloud_b->points.size ());

  float dist_h = std::max (max_dist_a, max_dist_b);
  float dist_rms = std::max (rms_dist_a , rms_dist_b);

  /////////////////////////////////////////////////////////
  //
  // calculate peak signal to noise ratio
/* Next code has been replaced by code derived from MPEG-3DGC pc_error
 * since the assumption in the next comment appeared not to be true.
 * However, it might be a good idea to work with normalized PC's only (xyz values scaled to (0,1),
 * both for (de)compression, icp, error calc. etc. and keep an inverse transform matrix
 * (concatenation of all operations) to be applied just before delivery of the PC's.
 
		// we assume the meshes are normalized per component and the peak energy therefore should approach 3
		//
		///////////////////////////////////////////////////////////
		PointT l_max_signal;
		PointT l_min_signal;
        const PointCloud<PointT> c_cloud_a(*cloud_a);

        // calculate peak geometric signal value
		getMinMax3D<PointT>(c_cloud_a,l_min_signal,l_max_signal);

		// calculate max energy of point
		float l_max_geom_signal_energy = l_max_signal.x * l_max_signal.x
			+ l_max_signal.y * l_max_signal.y + l_max_signal.z * l_max_signal.z ;
 */
  float l_max_geom_signal_energy = 0.0;
  if (quality_method_ & MAX_NN)
  {
    l_max_geom_signal_energy = max_dist_a_self*3;
  }
  else if (quality_method_ & TCSVT)
  {
    PointT l_max_signal;
    PointT l_min_signal;
    const PointCloud<PointT> c_cloud_a(*cloud_a);
      
    // calculate peak geometric signal value
    getMinMax3D<PointT>(c_cloud_a,l_min_signal,l_max_signal);
      
    // calculate max energy of point
    l_max_geom_signal_energy = l_max_signal.x * l_max_signal.x
      + l_max_signal.y * l_max_signal.y + l_max_signal.z * l_max_signal.z ;
  }
  float peak_signal_to_noise_ratio = 10 * std::log10( l_max_geom_signal_energy / (dist_rms * dist_rms));

  // compute averages for color distortions
  mse_colors_yuv[0] /= cloud_a->points.size ();
  mse_colors_yuv[1] /= cloud_a->points.size ();
  mse_colors_yuv[2] /= cloud_a->points.size ();

  // compute PSNR for YUV colors
  psnr_colors_yuv[0] = 10 * std::log10( 1.0 / mse_colors_yuv[0]);
  psnr_colors_yuv[1] = 10 * std::log10( 1.0 / mse_colors_yuv[1]);
  psnr_colors_yuv[2] = 10 * std::log10( 1.0 / mse_colors_yuv[2]);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms \n");
//print_info ("A->B: "); print_value ("%f\n", max_dist_a);
//print_info ("B->A: "); print_value ("%f\n", max_dist_b);
  print_info ("Symmetric Geometric Hausdorff Distance: "); print_value ("%f", dist_h);
  print_info (" ]\n\n");
//print_info ("A->B : "); print_value ("%f rms\n", rms_dist_a);
//print_info ("B->A: "); print_value ("%f rms\n", rms_dist_b);
  print_info ("Symmetric Geometric Root Mean Square Distance: "); print_value ("%f\n", dist_rms);
  print_info ("Geometric PSNR: "); print_value ("%f  dB", peak_signal_to_noise_ratio);
  print_info (" ]\n\n");
  print_info ("A->B color psnr Y: "); print_value ("%f dB ", (float) psnr_colors_yuv[0]);
  print_info ("A->B color psnr U: "); print_value ("%f dB ", (float) psnr_colors_yuv[1]);
  print_info ("A->B color psnr V: "); print_value ("%f dB ", (float) psnr_colors_yuv[2]);
  print_info (" ]\n");

  in_point_count_ = cloud_a->points.size ();
  out_point_count_ = cloud_b->points.size ();

  //////////////////////////////////////////////////////
  //
  // store the quality metrics in the return values
  //
  ////////////////////////////////////////////////////

  // geometric quality metrics [haussdorf Linf]
  left_hausdorff_ = max_dist_a;
  right_hausdorff_ = max_dist_b;
  symm_hausdorff_ = dist_h;

  // geometric quality metrics [rms L2]
  left_rms_ = rms_dist_a;
  right_rms_ = rms_dist_b;
  symm_rms_ = dist_rms;
  psnr_db_ = peak_signal_to_noise_ratio ;

  // color quality metric (non-symmetric, L2)
  psnr_yuv_[0]= psnr_colors_yuv[0];
  psnr_yuv_[1]= psnr_colors_yuv[1];
  psnr_yuv_[2]= psnr_colors_yuv[2];
}

//! function to print a header to csv file for logging the quality characteristics
void
QualityMetric::print_csv_line_(const std::string &compression_setting_arg, std::ostream &csv_ostream)
{
  csv_ostream << compression_setting_arg << std::string(";")
              << in_point_count_ << std::string(";")
			  << out_point_count_ << std::string(";")
			  << compressed_size_ << std::string(";")
			  << compressed_size_ / (1.0 * out_point_count_) << std::string(";")
			  << byte_count_octree_layer_ / (1.0 * out_point_count_) << std::string(";")
			  << byte_count_centroid_layer_ / (1.0 * out_point_count_) << std::string(";")
			  << byte_count_color_layer_ / (1.0 * out_point_count_) << std::string(";")
			  << symm_rms_ << std::string(";")
			  << symm_hausdorff_ << std::string(";")
			  << psnr_db_ << std::string(";")
			  << psnr_yuv_[0] << std::string(";")
			  << psnr_yuv_[1] << std::string(";")
			  << psnr_yuv_[2] << std::string(";")
			  << encoding_time_ms_ << std::string(";")
			  << decoding_time_ms_ << std::string(";")
			  << std::endl;
}

//! function to print a header to csv file for logging the quality characteristics
void
QualityMetric::print_csv_header_(std::ostream &csv_ostream)
{
  csv_ostream << "compression setting; "
			  << std::string("in point count;")
			  << std::string("out point count;")
			  << std::string("compressed_byte_size;")
			  << std::string("compressed_byte_size_per_output_point;")
			  << std::string("octree_byte_size_per_voxel;")
			  << std::string("centroid_byte_size_per_voxel;")
			  << std::string("color_byte_size_per_voxel;")
			  << std::string("symm_rms;")
			  << std::string("symm_haussdorff;")
			  << std::string("psnr_db;")
			  << std::string("psnr_colors_y;")
			  << std::string("psnr_colors_u;")
			  << std::string("psnr_colors_v;")
			  << std::string("encoding_time_ms;")
			  << std::string("decoding_time_ms;")
			  << std::endl;
}

#endif
