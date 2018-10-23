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

#ifndef QUALITY_METRIC_H
#define QUALITY_METRIC_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*!
	 *\brief
	 * enum to distinguish various methods of Geometric Peak Signal to Noise Ratio
	 * Since there is currently no clear agreement how to define geometric PSNR of degenerated
	 * Point Clouds (PCs) we define some variants, all based on the point-to-point approach:
	 * NONE	      - no quality computation
   * SELECT     - select quality computation with optional options
	 * ORIGINAL	  - Vorig-maxgeo     in https://github.com/RufaelDev/pcc-mp3dg/tree/tcsvt_version
   * TCSVT      - Vdegr-maxgeo in R.Mekuria e.a. IEEE TCSVT 277(4): pp. 828-842, 2017
	 * MAX_NN	  - same, but geom. peak value is maximal Nearest Neighbor distance
   * NORMALISED - PC x,y,z values re-scaled to [0,1), geom. peak value approaches sqrt(3)
	*/
	enum QualityMethod { NONE=0,        //! no quality computation
                       SELECT=1,      //! select quality computation with optional options
                       BBALIGNED=2,   //! compare bounding-box aligned pointclouds (i.o. original/final)
                       TCSVT=4,       //! Vdegr-maxgeo as in R.Mekuria e.a. IEEE TCSVT 277(4): pp. 828-842, 2017
                       MAX_NN=8,     //! same, but geom. peak value is maximal Nearest Neighbor distance
                       NORMALISED=16,  //! PC x,y,z values re-scaled to [0,1), geom. peak value approaches sqrt(3)
                       BT709=32       //! select BT709 i.o. BT.601
        };

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*!
	* \brief 
	*  struct to store achieved coding performance for official evaluation by MPEG committee (3DG)
	* \author Rufael Mekuria (rufael.mekuria@cwi.nl)
	*/
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
  class QualityMetric{
  public:
    QualityMethod quality_method_;
    std::size_t compressed_size_;    //! store the compressed byte size

    uint64_t in_point_count_;
    uint64_t out_point_count_;
    uint64_t byte_count_octree_layer_;
    uint64_t byte_count_centroid_layer_;
    uint64_t byte_count_color_layer_;

    float symm_rms_;                             //! store symm rms metric
    float symm_hausdorff_;                       //! store symm haussdorf
    float left_hausdorff_;                        //! store left haussdorf
    float right_hausdorff_;                      //! store right haussdorf
    float left_rms_;                             //! store left rms
    float right_rms_;                            //! store right rms
    double psnr_db_;                              //! store psnr for the geometry
    double psnr_yuv_[3];                          //! store psnr for the colors

    double encoding_time_ms_;
    double decoding_time_ms_;
      
    QualityMetric (QualityMethod method=NONE) :
      quality_method_(method),
      compressed_size_(0),
    
      in_point_count_(0),
      out_point_count_(0),
      byte_count_octree_layer_(0),
      byte_count_centroid_layer_(0),
      byte_count_color_layer_(0),
    
      symm_rms_(0),
      symm_hausdorff_(0),
      left_hausdorff_(0),
      right_hausdorff_(0),
      left_rms_(0),
      right_rms_(0),
      psnr_db_(0),
      psnr_yuv_(),
    
      encoding_time_ms_(0),
      decoding_time_ms_(0)
    {}

    template<typename PointT> void convertRGBtoYUV(const PointT &in_rgb, float * out_yuv);
      
    /*! \brief compute the quality metric, we assume the cloud_a is the original
     * and cloud_b the degenerated cloud
     */
    template<typename PointT>
    PCL_EXPORTS void
    computeQualityMetric (boost::shared_ptr<pcl::PointCloud<PointT> > cloud_a, boost::shared_ptr<pcl::PointCloud<PointT> > cloud_b);

    // return QualityMethod from string
    static PCL_EXPORTS QualityMethod
    get_QualityMethod (std::string& method_as_string);

    // print the header of a .csv file
    static PCL_EXPORTS void
    print_csv_header_ (std::ostream &csv_ostream);
	
    // print results into a .csv file
    PCL_EXPORTS void print_csv_line_ (const std::string &compression_setting_arg, std::ostream &csv_ostream);
  };


////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*!
 * function to determine a transformation matrix to fit a point cloud to fit in [min_box,max_box])
 * @param pc (input) cloud to be fitted
 * @param tm (output) transformation matrix
 * @param the lower corner of the box to fit the original pointcloud
 * @param the upper corner of the box to fit the original pointcloud
 * @param Procrustus (input) when true, perform anisotropic transformation (Procrustes style)
 * \note PointT typename of point used in point cloud
 * \author Kees Blom (Kees.Blom@cwi.nl) June 19, 2018.
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> PCL_EXPORTS
void
fitPointCloudInBox(boost::shared_ptr<pcl::PointCloud<PointT> > pc, Eigen::Matrix4f* rtm, PointT* min_box, PointT* max_box,
                   bool Procrustes=false);

#endif
