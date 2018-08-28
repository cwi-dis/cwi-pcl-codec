/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014, Stichting Centrum Wiskunde en Informatica.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*/
#ifndef POINT_CLOUD_CODEC_V2_OMP_H
#define POINT_CLOUD_CODEC_V2_OMP_H

// added for cloud codec v2
#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>

namespace pcl{

  namespace io{

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**! 
    * \brief class for computation and compression of time varying cloud frames in parallel, using the OpenMP standard.
    * Extends original PCL cloud codec_v2
    * \author Rufael Mekuria (rufael.mekuria@cwi.nl)
    */
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename PointT, typename LeafT = OctreeContainerPointIndices,
      typename BranchT = OctreeContainerEmpty,
      typename OctreeT = Octree2BufBase<LeafT, BranchT> >
    class OctreePointCloudCodecV2OMP : public OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>
    {
      public:
   
        // public typedefs, copied from original implementation by Julius Kammerl
       typedef typename OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::PointCloud PointCloud;
       typedef typename OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;
       typedef typename OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;
       typedef OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT> MacroBlockTree;

        // Boost shared pointers
        typedef boost::shared_ptr<OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT> > Ptr;
        typedef boost::shared_ptr<const OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT> > ConstPtr;

        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;
	/*
        typedef OctreePointCloudCodecV2<PointT, LeafT, BranchT, Octree2BufBase<LeafT, BranchT> > RealTimeStreamCompression;
        typedef OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeBase<LeafT, BranchT> > SinglePointCloudCompressionLowMemory;
      */
        /** \brief Constructor
        * \param compressionProfile_arg:  define compression profile
        * \param octreeResolution_arg:  octree resolution at lowest octree level
        * \param pointResolution_arg:  precision of point coordinates
        * \param doVoxelGridDownDownSampling_arg:  voxel grid filtering
        * \param iFrameRate_arg:  i-frame encoding rate
        * \param doColorEncoding_arg:  enable/disable color coding
        * \param colorBitResolution_arg:  color bit depth
        * \param showStatistics_arg:  output compression statistics
        * \param colorCodingType_arg:  jpeg or pcl dpcm
        * \param doVoxelGridCentroid_arg:  keep voxel grid positions or not 
        * \param createScalebleStream_arg:  scalable bitstream (not yet implemented)
        * \param codeConnectivity_arg:  connectivity coding (not yet implemented)
        * \param jpeg_quality_arg:  quality of the jpeg encoder (jpeg quality)
        */
        OctreePointCloudCodecV2OMP (compression_Profiles_e compressionProfile_arg = MED_RES_ONLINE_COMPRESSION_WITH_COLOR,
          bool showStatistics_arg = false,
          const double pointResolution_arg = 0.001,
          const double octreeResolution_arg = 0.01,
          bool doVoxelGridDownDownSampling_arg = false,
	  const unsigned int iFrameRate_arg = 0, /* NO PCL P Frames in this version of the codec !! */
          bool doColorEncoding_arg = true,
          const unsigned char colorBitResolution_arg = 6,
          const unsigned char colorCodingType_arg = 0,
			const unsigned char timeStamp = 0,
          bool doVoxelGridCentroid_arg = true, 
          bool createScalebleStream_arg = true, 
          bool codeConnectivity_arg = false,
          int jpeg_quality_arg = 75, 
          int num_threads=1) :
        OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>(
          compressionProfile_arg,
          showStatistics_arg,
          pointResolution_arg,
          octreeResolution_arg,
          doVoxelGridDownDownSampling_arg,
	  iFrameRate_arg, 
          doColorEncoding_arg,
          colorBitResolution_arg, 
          colorCodingType_arg, 
			timeStamp_arg,
          doVoxelGridCentroid_arg,
          createScalebleStream_arg,
          codeConnectivity_arg,
          jpeg_quality_arg),num_threads_(num_threads)
        {
        }

        void initialization ()
        {
        }

//      void
//      encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg);

//      void
//      decodePointCloud (std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg);

//      virtual void
//      generatePointCloudDeltaFrame (const PointCloudConstPtr &icloud_arg, const PointCloudConstPtr &pcloud_arg, PointCloudPtr &out_cloud_arg, 
//          std::ostream& i_coded_data, std::ostream& p_coded_data, bool icp_on_original = false,bool write_out_cloud = true );

        virtual void
        encodePointCloudDeltaFrame (const PointCloudConstPtr &icloud_arg, const PointCloudConstPtr &pcloud_arg, PointCloudPtr &out_cloud_arg, 
            std::ostream& i_coded_data, std::ostream& p_coded_data, bool icp_on_original = false,bool write_out_cloud = false);

//      virtual void
//      decodePointCloudDeltaFrame(const PointCloudConstPtr &icloud_arg, PointCloudPtr &out_cloud_arg, 
//          std::istream& i_coded_data, std::istream& p_coded_data);
// inherited protected members needed
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::do_icp_color_offset_;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::point_resolution_;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::octree_resolution_;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::color_bit_resolution_;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::color_coding_type_;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::do_voxel_centroid_enDecoding_;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::shared_macroblock_percentage_;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::shared_macroblock_convergence_percentage_;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::simplifyPCloud;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::generate_macroblock_tree;
        using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::do_icp_prediction;
		using pcl::io::OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::timeStamp;
//      using pcl::io:: OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::;
/*
        using pcl::octree::Octree2BufBase<LeafT, BranchT>::deleteCurrentBuffer;
        using pcl::octree::Octree2BufBase<LeafT, BranchT>::deserializeTree; // does not work in windows
        using pcl::octree::Octree2BufBase<LeafT, BranchT>::leaf_count_;
        using pcl::octree::Octree2BufBase<LeafT, BranchT>::serializeTree;
        using pcl::octree::Octree2BufBase<LeafT, BranchT>::switchBuffers;
        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointsFromInputCloud;
//        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::deleteCurrentBuffer;
        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::deleteTree;
        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getTreeDepth;
        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::input_;
        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::min_x_;
        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::min_y_;
        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::min_z_;
        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::resolution_;
        using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::setInputCloud;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::b_show_statistics_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::binary_tree_data_vector_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::cloud_with_color_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::color_bit_resolution_ ;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::compressed_color_data_len_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::compressed_point_data_len_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::color_coder_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::data_with_color_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::do_color_encoding_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::do_voxel_grid_enDecoding_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::entropy_coder_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::frame_ID_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::i_frame_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::i_frame_counter_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::i_frame_rate_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::object_count_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::octree_resolution_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::output_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::point_coder_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::point_color_offset_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::point_count_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::point_count_data_vector_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::point_count_data_vector_iterator_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::point_resolution_;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::setOutputCloud;
        using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::writeFrameHeader;
 
//      using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::jp_color_coder_;
//      using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::i_frame_counter_;
//      using pcl::io::OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::
//      using pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::
*/      
       //! number of omp threads
       int num_threads_;
    };

    // define frame identifier for cloud codec v2 omp
//  template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
//  const char* OctreePointCloudCodecV2OMP<PointT, LeafT, BranchT, OctreeT>::frame_header_identifier_ = "<PCL-OCT-CODECV2-COMPRESSED-OMP>";
  }
}
#endif // POINT_CLOUD_CODEC_V2_OMP_H
