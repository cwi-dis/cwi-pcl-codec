 /*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#ifndef QUALITY_METRICS_H
#define QUALITY_METRICS_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*!
	* \brief 
	*  struct to store achieved coding performance for official evaluation by MPEG committee (3DG)
	* \author Rufael Mekuria (rufael.mekuria@cwi.nl)
	*/
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
  struct QualityMetric{
	
	std::size_t compressed_size;    //! store the compressed byte size

	uint64_t in_point_count;
  uint64_t out_point_count;
	uint64_t byte_count_octree_layer;
	uint64_t byte_count_centroid_layer;
	uint64_t byte_count_color_layer;

  float symm_rms;                             //! store symm rms metric
	float symm_hausdorff;                       //! store symm haussdorf
	float left_hausdorff;                        //! store left haussdorf
	float right_hausdorff;                      //! store right haussdorf
	float left_rms;                             //! store left rms
	float right_rms;                            //! store right rms
	double psnr_db;                              //! store psnr for the geometry
	double psnr_yuv[3];                          //! store psnr for the colors

    double encoding_time_ms;
    double decoding_time_ms;

	// print the header of a .csv file
	static PCL_EXPORTS void print_csv_header(std::ostream &csv_ostream);
	
	// print results into a .csv file
	PCL_EXPORTS void print_csv_line (const std::string &compression_setting_arg, std::ostream &csv_ostream);
  };

  //! \brief compute the quality metric, we assume the cloud_a is the original (for colors we only compare the original to the lossy cloud)
  template<typename PointT> PCL_EXPORTS void
    computeQualityMetric (pcl::PointCloud<PointT>  &cloud_a, pcl::PointCloud<PointT>  &cloud_b, QualityMetric & qual_metric );

#endif
