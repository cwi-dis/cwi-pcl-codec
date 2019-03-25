/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2009-2012, Willow Garage, Inc.
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
#ifndef POINT_CLOUD_CODECV2_IMPL_HPP
#define POINT_CLOUD_CODECV2_IMPL_HPP
// STL Containers containing fixed-size vectorizable Eigen types need aligned_allocator<...>
// see: http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
#include <Eigen/StdVector>

// point cloud compression from PCL
#ifdef  PCL_INSTALLED
#include <pcl/io/impl/octree_pointcloud_compression.hpp>
#include <pcl/io/impl/entropy_range_coder.hpp>
#else //PCL_INSTALLED
#include <pcl/compression/impl/octree_pointcloud_compression.hpp>
#include <pcl/compression/impl/entropy_range_coder.hpp>
#endif//PCL_INSTALLED

#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>
#include <pcl/compression/point_coding.h>

#include <pcl/filters/radius_outlier_removal.h>
#if defined(_OPENMP)
#include <omp.h>
#endif//defined(_OPENMP)

//includes to do the ICP procedures
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Geometry>
#include <pcl/cloud_codec_v2/impl/rigid_transform_coding_impl.hpp>

namespace pcl{

  namespace io{

    /**
      * \brief  encoding routing based on overriding pcl octree codec written by Julius Kammerl
      *        extra features include
      *        - centroid coding
      *        - color coding based on jpeg
      *        - scalable bitstream (not yet implemented)
      * \author Rufael Mekuria rufael.mekuria@cwi.nl
      * \param  cloud_arg
      * \param  compressed_tree_data_out_arg
      */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void OctreePointCloudCodecV2<
      PointT, LeafT, BranchT, OctreeT>::encodePointCloud (
      const PointCloudConstPtr &cloud_arg,
      std::ostream& compressed_tree_data_out_arg)
    {
      unsigned char recent_tree_depth =
        static_cast<unsigned char> (getTreeDepth ());

      // CWI addition to prevent crashes as in original cloud codec
      deleteCurrentBuffer();
      deleteTree();

      // initialize octree
      setInputCloud (cloud_arg);

      // CWI added, when encoding hte output variable stores the (simplified icloud)
      output_ = PointCloudPtr(new PointCloud());

      // add point to octree
      addPointsFromInputCloud ();

      // make sure cloud contains points
      if (leaf_count_ > 0)
      {
        // color field analysis
        cloud_with_color_ = false;
        std::vector<pcl::PCLPointField> fields;
        int rgba_index = -1;
        rgba_index = pcl::getFieldIndex (*input_, "rgb", fields);
        if (rgba_index == -1)
        {
          rgba_index = pcl::getFieldIndex (*input_, "rgba", fields);
        }
        if (rgba_index >= 0)
        {
          point_color_offset_ = static_cast<unsigned char> (fields[rgba_index].offset);
          cloud_with_color_ = true;
        }

        // apply encoding configuration
        cloud_with_color_ &= do_color_encoding_;

        // if octree depth changed, we enforce I-frame encoding
        i_frame_ |= (recent_tree_depth != getTreeDepth ());// | !(iFrameCounter%10);

        // enable I-frame rate
        if (i_frame_counter_++==i_frame_rate_)
        {
          i_frame_counter_ =0;
          i_frame_ = true;
        }

        // increase frameID
        frame_ID_++;

        // do octree encoding
        if (!do_voxel_grid_enDecoding_)
        {
          point_count_data_vector_.clear ();
          point_count_data_vector_.reserve (cloud_arg->points.size ());
        }

        // initialize color encoding (including new color coding based on jpeg)
        if(!color_coding_type_){
          color_coder_.initializeEncoding ();
          color_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
          color_coder_.setVoxelCount (static_cast<unsigned int> (leaf_count_));
        }
        else
        {
          // new jpeg color coding
          jp_color_coder_.initializeEncoding ();
          jp_color_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
          jp_color_coder_.setVoxelCount (static_cast<unsigned int> (leaf_count_));
        }
        // initialize point encoding (including new centroid encoder)
        point_coder_.initializeEncoding ();
        centroid_coder_.initializeEncoding ();
        point_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
        centroid_coder_.initializeEncoding ();
        centroid_coder_.setPointCount( static_cast<unsigned int> (object_count_));

        // serialize octree
        if (i_frame_)
        {
          // i-frame encoding - encode tree structure without referencing previous buffer
          serializeTree (binary_tree_data_vector_, false);
        }
        else
        {
          // p-frame encoding - XOR encoded tree structure
          serializeTree (binary_tree_data_vector_, true);
        }

        // write frame header information to stream
        writeFrameHeader (compressed_tree_data_out_arg);

        // apply entropy coding to the content of all data vectors and send data to output stream
        entropyEncoding(compressed_tree_data_out_arg, compressed_tree_data_out_arg);

        // reset object count
        object_count_ = 0;

        if (b_show_statistics_) // todo update for codec v2
        {
          float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);
          float bytes_per_color = static_cast<float> (compressed_color_data_len_) / static_cast<float> (point_count_);
            
          PCL_INFO ("*** POINTCLOUD ENCODING ***\n");
          PCL_INFO ("Frame ID: %d\n", frame_ID_);
          if (i_frame_)
            PCL_INFO ("Encoding Frame: Intra frame\n");
          else
            PCL_INFO ("Encoding Frame: Prediction frame\n");
          PCL_INFO ("Number of encoded points: %ld\n", point_count_);
          PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof(float)) * 100.0f);
          PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
          PCL_INFO ("Color compression percentage: %f%%\n", bytes_per_color / (sizeof (int)) * 100.0f);
          PCL_INFO ("Color bytes per point: %f bytes\n", bytes_per_color);
          PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f  * sizeof (float)) / 1024);
          PCL_INFO ("Size of compressed point cloud: %d kBytes\n", (compressed_point_data_len_ + compressed_color_data_len_) / (1024));
          PCL_INFO ("Total bytes per point: %f\n", bytes_per_XYZ + bytes_per_color);
          PCL_INFO ("Total compression percentage: %f\n", (bytes_per_XYZ + bytes_per_color) / (sizeof (int) + 3.0f * sizeof(float)) * 100.0f);
          PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_color));
        }
      }
      else
      {
        if (b_show_statistics_) PCL_INFO ("Info: Dropping empty point cloud\n");
        deleteTree();
        i_frame_counter_ = 0;
        i_frame_ = true;
      }
    }

    /**
      * \brief  decoding routing based on overriding pcl octree codec written by Julius Kammerl
      *        extra features include
      *        - centroid coding
      *        - color coding based on jpeg
      * \author Rufael Mekuria rufael.mekuria@cwi.nl
      * \param  compressed_tree_data_in_arg
      * \param  cloud_arg  decoded point cloud
      */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> bool
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::decodePointCloud (
      std::istream& compressed_tree_data_in_arg,
      PointCloudPtr &cloud_arg, uint64_t &tmstmp)
    {
		//std::cout << "\n Entered cloud codec v2 \n";
		//std::cout << "\n Size of compressed frame" << sizeof(compressed_tree_data_in_arg);
      // synchronize to frame header
      if (!syncToHeader(compressed_tree_data_in_arg)) return false;
	  //std::cout << "\n Header synced \n";
      // initialize octree
      switchBuffers ();

      // added to prevent crashes as happens with original cloud codec
      deleteCurrentBuffer();
      deleteTree();
      setOutputCloud (cloud_arg);
	  //std::cout << " \n Octree buffers switched \n";
      // color field analysis
      cloud_with_color_ = false;
      std::vector<pcl::PCLPointField> fields;
      int rgba_index = -1;
      rgba_index = pcl::getFieldIndex (*output_, "rgb", fields);
      if (rgba_index == -1)
        rgba_index = pcl::getFieldIndex (*output_, "rgba", fields);
      if (rgba_index >= 0)
      {
        point_color_offset_ = static_cast<unsigned char> (fields[rgba_index].offset);
        cloud_with_color_ = true;
      }

      // read header from input stream
      readFrameHeader (compressed_tree_data_in_arg);
	  tmstmp = timeStamp;
	  //TO DO Shishir: Set codec parameteres from frame header before decoding
	  //
	  //this->octree_resolution_ = octreeResolution;
	  //this->color_bit_resolution_ = colorBitResolution;
	  //this->jpeg_quality_ = jpeg_quality;

	  //std::cout << "\n Frame header read \n ";
      // set the right grid pattern to the JPEG coder
      jp_color_coder_ = ColorCodingJPEG<PointT>(75,color_coding_type_);

      // decode data vectors from stream
      entropyDecoding(compressed_tree_data_in_arg, compressed_tree_data_in_arg);
	  //std::cout << "\n Entropy coding done \n";
      // initialize color and point encoding
      if(!color_coding_type_)
        color_coder_.initializeDecoding ();
      else
        jp_color_coder_.initializeDecoding ();

      point_coder_.initializeDecoding ();
      centroid_coder_.initializeDecoding();

      // initialize output cloud
      output_->points.clear ();
      output_->points.reserve (static_cast<std::size_t> (point_count_));
	  //std::cout << "\n Starting to deserialize \n";
      if (i_frame_)
        // i-frame decoding - decode tree structure without referencing previous buffer
        deserializeTree (binary_tree_data_vector_, false);
      else
        // p-frame decoding - decode XOR encoded tree structure
        deserializeTree (binary_tree_data_vector_, true);

      // assign point cloud properties
      output_->height = 1;
      output_->width = static_cast<uint32_t> (cloud_arg->points.size ());
      output_->is_dense = false;
	  //std::cout << "\n Pointcloud properties assigned \n";
      if (b_show_statistics_)
      {
        float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);
        float bytes_per_color = static_cast<float> (compressed_color_data_len_) / static_cast<float> (point_count_);

        PCL_INFO ("*** POINTCLOUDV2 DECODING ***\n");
        PCL_INFO ("Frame ID: %d\n", frame_ID_);
        if (i_frame_)
          PCL_INFO ("Encoding Frame: Intra frame\n");
        else
          PCL_INFO ("Encoding Frame: Prediction frame\n");
        PCL_INFO ("Number of encoded points: %ld\n", point_count_);
        PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
        PCL_INFO ("Color compression percentage: %f%%\n", bytes_per_color / (sizeof (int)) * 100.0f);
        PCL_INFO ("Color bytes per point: %f bytes\n", bytes_per_color);
        PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
        PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_ + compressed_color_data_len_) / 1024.0f);
        PCL_INFO ("Total bytes per point: %d bytes\n", static_cast<int> (bytes_per_XYZ + bytes_per_color));
        PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ + bytes_per_color) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_color));
    }
	  //std::cout << "\nDecoding done\n";
    return true;
  }

    /**
      * \brief  helper function to compute the delta frames
      * \author Rufael Mekuria rufael.mekuria@cwi.nl
      * \param  pcloud_arg_in input point cloud to simplify
      * \param  out_cloud  output simplified point cloud
      */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>::simplifyPCloud(const PointCloudConstPtr &pcloud_arg_in, 
      PointCloudPtr &out_cloud )
    {
      // this is the octree coding part of the predictive encoding
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT> *octree_simplifier = new OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>
        (
        MANUAL_CONFIGURATION,
        false,
        point_resolution_,
        octree_resolution_,
        true,
        0,
        true,
        color_bit_resolution_   /*,0,do_voxel_centroid_enDecoding_*/
        );

      // use bounding box and obtain the octree grid
      octree_simplifier->defineBoundingBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
      octree_simplifier->setInputCloud (pcloud_arg_in);
      octree_simplifier->addPointsFromInputCloud ();

      /////////////////////////////////////////////////////////////////////////
      //! initialize output cloud
      //PointCloudPtr out_cloud(new PointCloud( octree_simplifier.leaf_count_, 1));
      out_cloud->width = (uint32_t) octree_simplifier->getLeafCount();
      out_cloud->height = 1;
      out_cloud->points.reserve(octree_simplifier->getLeafCount());  
      // cant get access to the number of leafs
      ////////////////////////////////////////////////////////////////////////////

      ///////////// compute the simplified cloud by iterating the octree ////////
#if PCL_VERSION >= 100901
	octree::OctreeDepthFirstIterator<OctreeT> it_ = octree_simplifier->leaf_depth_begin();

	  for (int l_index = 0; *it_; it_++, l_index++)
	  {
		  if (it_.isBranchNode()) continue;
#else//PCL_VERSION >= 100901
	  octree::OctreeDepthFirstIterator<OctreeT> it_ = octree_simplifier->leaf_begin();
	  octree::OctreeLeafNodeIterator<OctreeT> it_end = octree_simplifier->leaf_end();

      for(int l_index =0;it_ !=it_end; it_++, l_index++)
      {
#endif//PCL_VERSION >= 100901
        // new point for the simplified cloud
        PointT l_new_point;

        //! centroid for storage
        std::vector<int>& point_indices = it_.getLeafContainer().getPointIndicesVector();

        // if centroid coding, store centroid, otherwise add 
        if(!do_voxel_centroid_enDecoding_)
        {
          octree_simplifier->genLeafNodeCenterFromOctreeKey(it_.getCurrentOctreeKey(),l_new_point);
        }
        else
        {
          Eigen::Vector4f cent;
          pcl::compute3DCentroid<PointT>(*pcloud_arg_in, point_indices, cent);

          l_new_point.x = cent[0];
          l_new_point.y = cent[1];
          l_new_point.z = cent[2];
        }
        long color_r=0;
        long color_g=0;
        long color_b=0;

        //! compute average color
        for(int i=0; i< point_indices.size();i++)
        {
          color_r+=pcloud_arg_in->points[point_indices[i]].r;
          color_g+=pcloud_arg_in->points[point_indices[i]].g;
          color_b+=pcloud_arg_in->points[point_indices[i]].b;
        }

        l_new_point.r = (char) (color_r / point_indices.size());
        l_new_point.g = (char) (color_g / point_indices.size());
        l_new_point.b = (char) (color_b / point_indices.size());

        out_cloud->points.push_back(l_new_point);
      }
      //////////////// done computing simplified cloud and octree structure //////////////////
      delete octree_simplifier;
    };
    /*!
    * \brief  helper function to  generate macroblock tree for computing shared macroblocks
    * \author Rufael Mekuria rufael.mekuria@cwi.nl
    * \param  in_cloud input point cloud
    * \return pointer to a macroblock tree structure
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT> *  
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>::generate_macroblock_tree(PointCloudConstPtr  in_cloud)
    {
      MacroBlockTree *tree = new MacroBlockTree
        (
        MANUAL_CONFIGURATION,
        false,
        point_resolution_,
        octree_resolution_ * macroblock_size_,
        true,
        0,
        true,
        color_bit_resolution_
        /*,0,do_voxel_centroid_enDecoding_*/
        );

      // I frame coder
      tree->defineBoundingBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
      tree->setInputCloud (in_cloud); /* assume right argument was given, either the original p cloud or an i cloud */
      tree->addPointsFromInputCloud ();
      return tree;
    }

    /*!
    * \brief  helper function to  do ICP prediction between shared macroblocks
    * \author Rufael Mekuria rufael.mekuria@cwi.nl
    * \param  i_cloud macroblock icloud
    * \param  p_cloud macroblock pcloud
    * \param  rigid_transform  output rigid transform
    * \param  has_converged  true if converged otherwise false
    * \param  rgb_offsets values for rgb offsets (if enabled)
    * pointer to a macroblock tree structure
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void 
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>::do_icp_prediction(
	Eigen::Matrix4f& rigid_transform,
	PointCloudPtr i_cloud,
	PointCloudPtr p_cloud,
	bool & has_converged,
	char *rgb_offsets
      )
    {
      // check if it makes sense to do icp, i.e. input and output points approx. equal
      bool do_icp =  p_cloud->size() > 6 ? 
        (p_cloud->size() < i_cloud->size() * 2) 
        &&  (p_cloud->size() >= i_cloud->size() * 0.5) : false;

      if(!do_icp)
      {
        has_converged = false;
        return;
      }

      // color variance filter
      double in_av[3]={0,0,0};
      double out_av[3]={0,0,0};
      double in_var=0;
      double out_var=0;

      if(do_icp)
      {
        // create references to ease acccess via [] operator in the cloud
        pcl::PointCloud<PointT> & rcloud_out = *p_cloud;
        pcl::PointCloud<PointT> & rcloud_in = *i_cloud;

        for(int i=0; i<rcloud_in.size();i++)
        {
          in_av[0]+= (double) rcloud_in[i].r;
          in_av[1]+= (double) rcloud_in[i].g;
          in_av[2]+= (double) rcloud_in[i].b;
        }

        in_av[0]/=rcloud_in.size();
        in_av[1]/=rcloud_in.size();
        in_av[2]/=rcloud_in.size();

        // variance
        for(int i=0; i<rcloud_in.size();i++)
        {
          double val= (rcloud_in[i].r - in_av[0]) * (rcloud_in[i].r - in_av[0]) + 
            (rcloud_in[i].g - in_av[1]) * (rcloud_in[i].g - in_av[1]) +
            (rcloud_in[i].b - in_av[2]) * (rcloud_in[i].b - in_av[2]);

          in_var+=val;
        }
        in_var/=(3*rcloud_in.size());

        //
        for(int i=0; i<rcloud_out.size();i++)
        {
          out_av[0]+= (double) rcloud_out[i].r;
          out_av[1]+= (double) rcloud_out[i].g;
          out_av[2]+= (double) rcloud_out[i].b;
        }
        out_av[0]/=rcloud_out.size();
        out_av[1]/=rcloud_out.size();
        out_av[2]/=rcloud_out.size();

        for(int i=0; i<rcloud_out.size();i++)
        {
          double val= (rcloud_out[i].r - out_av[0]) * (rcloud_out[i].r - out_av[0]) + 
            (rcloud_out[i].g - out_av[1]) * (rcloud_out[i].g - out_av[1]) +
            (rcloud_out[i].b - out_av[2]) * (rcloud_out[i].b - out_av[2]);

          out_var+=val;
        }
        out_var/=(3*rcloud_out.size());

        // for segments with large variance, skip the icp prediction
        if(in_var > icp_var_threshold_ || out_var > icp_var_threshold_)
          do_icp = false;
      }


      char &rgb_offset_r=rgb_offsets[0]; 
      char &rgb_offset_g=rgb_offsets[1]; 
      char &rgb_offset_b=rgb_offsets[2]; 

      if(do_icp_color_offset_){
        if(std::abs(out_av[0] - in_av[0]) < 32)
          rgb_offset_r = (char)(out_av[0] - in_av[0]);
        if(std::abs(out_av[1] - in_av[1]) < 32)
          rgb_offset_g = (char)(out_av[1] - in_av[1]);
        if(std::abs(out_av[2] - in_av[2]) < 32)
          rgb_offset_b = (char)(out_av[2] - in_av[2]);
      }

      if(!do_icp)
      {
        has_converged = false;
        return;
      }

      // do the actual icp transformation
      pcl::IterativeClosestPoint<PointT, PointT> icp;

      icp.setInputSource(i_cloud);
      icp.setInputTarget(p_cloud);

      icp.setMaximumIterations (icp_max_iterations_);
      // Set the transformation epsilon (criterion 2)
      icp.setTransformationEpsilon (transformationepsilon_);
      // Set the euclidean distance difference epsilon (criterion 3)
      icp.setEuclideanFitnessEpsilon (3 * transformationepsilon_);

      pcl::PointCloud<PointT> Final;
      icp.align(Final);

      // compute the fitness for the colors

      if(icp.hasConverged() && icp.getFitnessScore() < point_resolution_ * 2)
      {
        has_converged = true;
//        rigid_transform = new Eigen::Matrix4f(icp.getFinalTransformation());
	rigid_transform = icp.getFinalTransformation();
        return;
      }
      has_converged = false;
    }

    /*! \brief generate a point cloud Delta to output stream
    * \param icloud_arg  point cloud to be used a I frame
    * \param pcloud_arg  point cloud to be encoded as a pframe
    * \param out_cloud_arg [out] the predicted frame 
    * \param i_coded_data intra encoded data 
    * \param p_coded_data inter encoded data
    * \param icp_on_original  (option to do icp on original or simplified clouds)
    * \param write_out_cloud  (flag to write the output cloud to out_cloud_arg)
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void 
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>::generatePointCloudDeltaFrame( 
      const PointCloudConstPtr &icloud_arg /* icloud is already stored but can also be given as argument */,
      const PointCloudConstPtr &pcloud_arg,
      PointCloudPtr& out_cloud_arg, /* write the output cloud so we can assess the quality resulting from this algorithm */
      std::ostream& i_coded_data, 
      std::ostream& p_coded_data, 
      bool icp_on_original,
      bool write_out_cloud)
    {
      // intra coded points storage (points that cannot be predicted)
      typename pcl::PointCloud<PointT>::Ptr intra_coded_points(new pcl::PointCloud<PointT>());

      // initialize simplified predictive cloud
      PointCloudPtr simp_pcloud(new PointCloud());

      if(!icp_on_original){
        simplifyPCloud(pcloud_arg, simp_pcloud);
      }

      // keep track of the prediction statistics
      long macro_block_count=0;
      long shared_macroblock_count=0;
      long convergence_count=0;

      // initialize output cloud
      out_cloud_arg->height=1;
      out_cloud_arg->width =0;

      ////////////// generate the octree for the macroblocks //////////////////////////////
      MacroBlockTree * i_block_tree = generate_macroblock_tree(icloud_arg);
      MacroBlockTree * p_block_tree = generate_macroblock_tree(icp_on_original ? pcloud_arg:simp_pcloud);

      //////////// iterate the predictive frame and find common macro blocks /////////////
#if PCL_VERSION >= 100901
	  octree::OctreeDepthFirstIterator<OctreeT> it_predictive = p_block_tree->leaf_begin();

	  for (; *it_predictive; ++it_predictive)
	  {
	    if (it_predictive.isBranchNode()) continue;
#else//PCL_VERSION >= 100901
	  octree::OctreeDepthFirstIterator<OctreeT> it_predictive = p_block_tree->leaf_begin();
	  octree::OctreeDepthFirstIterator<OctreeT> it_predictive_end = p_block_tree->leaf_end();

      for(;it_predictive!=it_predictive_end;++it_predictive)
      {
#endif//PCL_VERSION >= 100901
        macro_block_count++;
        const octree::OctreeKey current_key = it_predictive.getCurrentOctreeKey();
        pcl::octree::OctreeContainerPointIndices *i_leaf;
        typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>(icp_on_original ? *pcloud_arg : *simp_pcloud , it_predictive.getLeafContainer().getPointIndicesVector()));

        if((i_leaf = i_block_tree->findLeaf(current_key.x,current_key.y,current_key.z)) != NULL)
        {
          typename pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>(*icloud_arg, i_leaf->getPointIndicesVector()));
          shared_macroblock_count++;
          // shared block, do icp
          bool icp_success=false;
          Eigen::Matrix4f rt = Eigen::Matrix4f::Identity();
          char rgb_offsets[3]={0,0,0};

          do_icp_prediction(
            rt,
            cloud_in,
            cloud_out,
            icp_success,
            rgb_offsets
            );

          if(icp_success)
          {
            convergence_count++;


            // icp success, encode the rigid transform
            std::vector<int16_t> comp_dat;
            Eigen::Quaternion<float> l_quat_out;
            RigidTransformCoding<float>::compressRigidTransform(rt,comp_dat,l_quat_out);

            // write octree key, write rigid transform
            int16_t l_key_dat[3]={0,0,0};

            l_key_dat[0] = (int) current_key.x; 
            l_key_dat[1] = (int) current_key.y; 
            l_key_dat[2] = (int) current_key.z;

            // write the p coded data (we can add entropy encoding later)
            p_coded_data.write((const char *) l_key_dat ,3*sizeof(int16_t));
            p_coded_data.write((const char *) &comp_dat[0] ,comp_dat.size()*sizeof(int16_t));

            if(do_icp_color_offset_)
              p_coded_data.write((const char *) &rgb_offsets[0] ,3*sizeof(char));

            // following code is for generation of the predicted frame
            Eigen::Matrix4f mdec = Eigen::Matrix4f::Identity();
            Eigen::Quaternion<float> l_quat_out_dec;
            RigidTransformCoding<float>::deCompressRigidTransform(comp_dat, mdec,l_quat_out_dec);

            /* uncomment this code for debugging the rigid transform
            bool corrected_tf_matrix=false;
            for(int i=0; i<16;i++)
            {
            float diff=0;
            if((diff=std::abs(rt(i/4,i%4) - mdec(i/4,i%4))) > 0.01){
            //mdec(i/4,i%4) = rt(i/4,i%4);
            corrected_tf_matrix=true;
            std::cout << " error decoding rigid transform "
            << comp_dat.size() << " index " << i/4 << "," 
            << i%4 << std::endl;
            std::cout << " original " << rt << std::endl;
            std::cout << " decoded " <<  mdec << std::endl;
            std::cin.get();
            }
            }
            if(corrected_tf_matrix)
            std::cout << " matrix decoded not ok " <<std::endl;
            else
            std::cout << " matrix decoded ok " <<std::endl;
            */
            if(write_out_cloud){
              // predicted point cloud
              pcl::PointCloud<PointT> manual_final;
              transformPointCloud<PointT, float>
                (  *cloud_in,
                manual_final, 
                mdec
                );

              // generate the output points
              for(int i=0; i < manual_final.size();i++){

                PointT &pt = manual_final[i];

                // color offset
                if(do_icp_color_offset_){
                  pt.r+=rgb_offsets[0];
                  pt.g+=rgb_offsets[1];
                  pt.b+=rgb_offsets[2];
                }

                out_cloud_arg->push_back(pt);
              }
            }
          }
          else
          {
            // icp failed
            // add to intra coded points
            for(int i=0; i < cloud_out->size();i++){
              if(write_out_cloud)
                out_cloud_arg->push_back((*cloud_out)[i]);
              intra_coded_points->push_back((*cloud_out)[i]);
            }
          }
        }
        else
        {
          // exclusive block
          // add to intra coded points
          for(int i=0; i < cloud_out->size();i++)
          {
            if(write_out_cloud)
              out_cloud_arg->push_back((*cloud_out)[i]);
            intra_coded_points->push_back((*cloud_out)[i]);
          }
        }
      }  
      /* encode all the points that could not be predicted 
      from the previous coded frame in i frame manner with cluod_codec_v2 */
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT> intra_coder 
        (
        MANUAL_CONFIGURATION,
        b_show_statistics_,
        point_resolution_,
        octree_resolution_,
        true,
        0,
        true,
        color_bit_resolution_,
        color_coding_type_, 
			timeStamp,
        do_voxel_centroid_enDecoding_
        );

      intra_coder.defineBoundingBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
      intra_coder.encodePointCloud(intra_coded_points, i_coded_data);

      //compute convergence statistics
      //compute the convergence statistics
      shared_macroblock_percentage_ =  (float)shared_macroblock_count / (float)macro_block_count ;
      shared_macroblock_convergence_percentage_ =  (float)convergence_count / (float)shared_macroblock_count;

      // clean up
      delete i_block_tree;
      delete p_block_tree;
    }
    // Next data structures are used to store essential information to parallelize a CPU intensive loop
	template<typename T> struct cloudInfoT { typename  pcl::octree::OctreeContainerPointIndices* i_leaf; octree::OctreeKey current_key; std::vector<int>* indices; };
	template<typename T> struct cloudResultT { Eigen::Matrix4f rt; typename pcl::PointCloud<T>::Ptr in_cloud; typename pcl::PointCloud<T>::Ptr out_cloud; bool icp_success; char rgb_offsets[3]; bool leaf_found; };
    /** \brief routine to encode a delta frame (predictive coding)
    * \author Rufael Mekuria rufael.mekuria@cwi.nl
    * \param icloud_arg  point cloud to be used a I frame
    * \param pcloud_arg  point cloud to be encoded as a pframe
    * \param out_cloud_arg [out] the predicted frame
    * \param i_coded_data intra encoded data
    * \param p_coded_data inter encoded data
    * \param icp_on_original  (option to do icp on original or simplified clouds)
    * \param write_out_cloud  (flag to write the output cloud to out_cloud_arg)
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
		OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::encodePointCloudDeltaFrame(
			const PointCloudConstPtr &icloud_arg, const PointCloudConstPtr &pcloud_arg, PointCloudPtr &out_cloud_arg,
			std::ostream& i_coded_data, std::ostream& p_coded_data, bool icp_on_original, bool write_out_cloud)
	{
#ifndef _OPENMP
		num_threads_ = 0;
#endif// _OPENMP
        std::vector<cloudInfoT<PointT> > p_info_list;
        std::vector<cloudResultT<PointT> > p_result_list;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > p_result_matrices;

		// intra coded points storage (points that cannot be predicted)
		typename pcl::PointCloud<PointT>::Ptr intra_coded_points(new pcl::PointCloud<PointT>());

		// keep track of the prediction statistics
		long macro_block_count = 0;
		long shared_macroblock_count = 0;
		long convergence_count = 0;

		// initialize predictive cloud
		PointCloudPtr simp_pcloud(new PointCloud());

		if (!icp_on_original) {
			simplifyPCloud(pcloud_arg, simp_pcloud);
		}

		// initialize output cloud
		out_cloud_arg->height = 1;
		out_cloud_arg->width = 0;

		////////////// generate the octree for the macroblocks //////////////////////////////
		MacroBlockTree * i_block_tree = generate_macroblock_tree(icloud_arg);
		MacroBlockTree * p_block_tree = generate_macroblock_tree(icp_on_original ? pcloud_arg : simp_pcloud);

		//////////// iterate the predictive frame and find common macro blocks /////////////
#if PCL_VERSION >= 100901
		octree::OctreeDepthFirstIterator<OctreeT> it_predictive = p_block_tree->leaf_depth_begin();
		if (num_threads_ == 0)
		{
			for (; *it_predictive; ++it_predictive)
			{
				if (!(it_predictive.isLeafNode())) continue;
#else//PCL_VERSION >= 100901
		octree::OctreeDepthFirstIterator<OctreeT> it_predictive = p_block_tree->leaf_begin();
		octree::OctreeDepthFirstIterator<OctreeT> it_predictive_end = p_block_tree->leaf_end();
		if (num_threads_ == 0)
		{
			for (; it_predictive != it_predictive_end; ++it_predictive)
			{
#endif//PCL_VERSION >= 100901
					const octree::OctreeKey current_key = it_predictive.getCurrentOctreeKey();
				pcl::octree::OctreeContainerPointIndices *i_leaf;
				typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>(icp_on_original ? *pcloud_arg : *simp_pcloud, it_predictive.getLeafContainer().getPointIndicesVector()));
				macro_block_count++;

				if ((i_leaf = i_block_tree->findLeaf(current_key.x, current_key.y, current_key.z)) != NULL)
				{
					shared_macroblock_count++;
					typename pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>(*icloud_arg, i_leaf->getPointIndicesVector()));

					// shared block, do icp
					bool icp_success = false;
					Eigen::Matrix4f rt = Eigen::Matrix4f::Identity();
					char rgb_offsets[3] = { 0,0,0 };

					// Next function is CPU intensive, and candidate for parallel execution
					do_icp_prediction(
						rt,
						cloud_in,
						cloud_out,
						icp_success,
						rgb_offsets
						);
					if (icp_success)
					{
						convergence_count++;
						// icp success, encode the rigid transform
						std::vector<int16_t> comp_dat;
						Eigen::Quaternion<float> l_quat_out;
						RigidTransformCoding<float>::compressRigidTransform(rt, comp_dat, l_quat_out);

						// write octree key, write rigid transform
						int16_t l_key_dat[3] = { 0,0,0 };

						l_key_dat[0] = (int)current_key.x;
						l_key_dat[1] = (int)current_key.y;
						l_key_dat[2] = (int)current_key.z;

						// write the p coded data (we can add entropy encoding later)
						uint8_t chunk_size = (uint8_t)(3 * sizeof(int16_t) + comp_dat.size()*sizeof(int16_t) + (do_icp_color_offset_ ? 3 : 0)); // size of the chunk
						p_coded_data.write((const char *)&chunk_size, sizeof(chunk_size));
						p_coded_data.write((const char *)l_key_dat, 3 * sizeof(int16_t));
						p_coded_data.write((const char *)&comp_dat[0], comp_dat.size()*sizeof(int16_t));

						if (do_icp_color_offset_)
							p_coded_data.write((const char *)rgb_offsets, 3 * sizeof(char));

						// following code is for generation of predicted frame
						Eigen::Matrix4f mdec = Eigen::Matrix4f::Identity();
						Eigen::Quaternion<float> l_quat_out_dec;
						RigidTransformCoding<float>::deCompressRigidTransform(comp_dat, mdec, l_quat_out_dec);

						// predicted point cloud
						pcl::PointCloud<PointT> manual_final;
						if (write_out_cloud) {
							transformPointCloud<PointT, float>
								(*cloud_in,
									manual_final,
									mdec
									);
						}

						// generate the output points
						if (write_out_cloud) {
							for (int i = 0; i < manual_final.size(); i++) {

								PointT &pt = manual_final[i];

								// color offset
								if (do_icp_color_offset_) {
									pt.r += rgb_offsets[0];
									pt.g += rgb_offsets[1];
									pt.b += rgb_offsets[2];
								}
								out_cloud_arg->push_back(pt);
							}
						}
					}
					else
					{
						// icp failed
						// add to intra coded points
						for (int i = 0; i < cloud_out->size(); i++) {
							if (write_out_cloud)
								out_cloud_arg->push_back((*cloud_out)[i]);
							intra_coded_points->push_back((*cloud_out)[i]);
						}
					}
				}
				else
				{
					// exclusive block
					// add to intra coded points
					for (int i = 0; i < cloud_out->size(); i++)
					{
						if (write_out_cloud)
							out_cloud_arg->push_back((*cloud_out)[i]);
						intra_coded_points->push_back((*cloud_out)[i]);
					}
				}
			}
		}
		else // num_threads > 0 --> parallelization
/* For _OPENMP, the previous loop over the common macro blocks of a frame must be converted in an equivalent one,
* where the number of iterations is known before the loop starts.
* We do this by storing all information in the current loop that is needed for the computations in the vector p_info_list,
* next start another loop in parallel chunks managed by OPENMP for the actual computations, storing the results in the vector p_result_list.
* Finally, in a third loop is all results are further processed as in the sequential setting.
*/
		{
			// store the input arguments for 'do_icp_prediction'
			for (; *it_predictive; ++it_predictive) {
				if (it_predictive.isBranchNode()) continue;
				const octree::OctreeKey current_key = it_predictive.getCurrentOctreeKey();
				pcl::octree::OctreeContainerPointIndices* i_leaf = i_block_tree->findLeaf(current_key.x, current_key.y, current_key.z);
				cloudInfoT<PointT> ci;
				cloudResultT<PointT> cr;

				ci.i_leaf = i_leaf;
				ci.current_key = current_key;
				ci.indices = &it_predictive.getLeafContainer().getPointIndicesVector();
				cr.leaf_found = i_leaf != NULL;
				cr.icp_success = false;
				for (int j = 0; j < 3; j++) {
					cr.rgb_offsets[j] = 0;
				}
				p_info_list.push_back(ci);
				p_result_list.push_back(cr);
				Eigen::Matrix4f crm;
				p_result_matrices.push_back(crm);
				macro_block_count++;
			}
#if defined(_OPENMP)
			std::cout << " running in parallel on " << num_threads_ << std::endl;
			omp_set_num_threads(num_threads_);
#endif//defined(_OPENMP)
#pragma omp parallel for shared(p_info_list,p_result_list,p_result_matrices)
			for (int i = 0; i < p_info_list.size(); i++) {
				pcl::octree::OctreeContainerPointIndices* i_leaf = p_info_list[i].i_leaf;
				typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>(icp_on_original ? *pcloud_arg : *simp_pcloud, *p_info_list[i].indices));
				p_result_list[i].out_cloud = cloud_out;
				if (i_leaf != NULL) {
					p_result_list[i].in_cloud = (PointCloudPtr) new pcl::PointCloud<PointT>(*icloud_arg, i_leaf->getPointIndicesVector());
					do_icp_prediction(
						p_result_matrices[i],
						(PointCloudPtr)p_result_list[i].in_cloud,
						(PointCloudPtr)p_result_list[i].out_cloud,
						p_result_list[i].icp_success,
						p_result_list[i].rgb_offsets
						);
				}
			} // #pragma omp for
#pragma omp barrier // wait until all threads finished
			for (int i = 0; i < p_result_list.size(); i++)
			{
				typename pcl::PointCloud<PointT>::Ptr cloud_out = p_result_list[i].out_cloud;
				if (p_result_list[i].leaf_found)
				{
					shared_macroblock_count++;
					typename pcl::PointCloud<PointT>::Ptr cloud_in = p_result_list[i].in_cloud;
					if (p_result_list[i].icp_success)
					{
						char rgb_offsets[3];
						Eigen::Matrix4f rt = p_result_matrices[i];
						for (int j = 0; j < 3; j++) {
							rgb_offsets[j] = p_result_list[i].rgb_offsets[j];
						}
						octree::OctreeKey current_key = p_info_list[i].current_key;

						convergence_count++;
						// icp success, encode the rigid transform
						std::vector<int16_t> comp_dat;
						Eigen::Quaternion<float> l_quat_out;
						RigidTransformCoding<float>::compressRigidTransform(rt, comp_dat, l_quat_out);

						// write octree key, write rigid transform
						int16_t l_key_dat[3] = { 0, 0, 0 };

						l_key_dat[0] = (int)current_key.x;
						l_key_dat[1] = (int)current_key.y;
						l_key_dat[2] = (int)current_key.z;

						// write the p coded data (we can add entropy encoding later)
						uint8_t chunk_size = (uint8_t)(3 * sizeof(int16_t) + comp_dat.size()*sizeof(int16_t) + (do_icp_color_offset_ ? 3 : 0)); // size of the chunk
						p_coded_data.write((const char *)&chunk_size, sizeof(chunk_size));
						p_coded_data.write((const char *)l_key_dat, 3 * sizeof(int16_t));
						p_coded_data.write((const char *)&comp_dat[0], comp_dat.size()*sizeof(int16_t));

						if (do_icp_color_offset_)
							p_coded_data.write((const char *)rgb_offsets, 3 * sizeof(char));

						// following code is for generation of predicted frame
						Eigen::Matrix4f mdec;
						Eigen::Quaternion<float> l_quat_out_dec;
						RigidTransformCoding<float>::deCompressRigidTransform(comp_dat, mdec, l_quat_out_dec);

						// predicted point cloud
						pcl::PointCloud<PointT> manual_final;
						if (write_out_cloud) {
							transformPointCloud<PointT, float>
								(*cloud_in,
									manual_final,
									mdec
									);
						}

						// generate the output points
						if (write_out_cloud) {
							for (int i = 0; i < manual_final.size(); i++) {

								PointT &pt = manual_final[i];

								// color offset
								if (do_icp_color_offset_) {
									pt.r += rgb_offsets[0];
									pt.g += rgb_offsets[1];
									pt.b += rgb_offsets[2];
								}

								out_cloud_arg->push_back(pt);
							}
						}
					}
					else
					{
						// icp failed
						// add to intra coded points
						for (int i = 0; i < cloud_out->size(); i++) {
							if (write_out_cloud)
								out_cloud_arg->push_back((*cloud_out)[i]);
							intra_coded_points->push_back((*cloud_out)[i]);
						}
					}
				}
				else
				{
					// exclusive block
					// add to intra coded points
					for (int i = 0; i < cloud_out->size(); i++)
					{
						if (write_out_cloud)
							out_cloud_arg->push_back((*cloud_out)[i]);
						intra_coded_points->push_back((*cloud_out)[i]);
					}
				}
			}
		}
		

		/* encode all the points that could not be predicted
		from the previous coded frame in i frame manner with cloud_codec_v2 */
		OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT> intra_coder
			(
				MANUAL_CONFIGURATION,
				false,
				point_resolution_,
				octree_resolution_,
				true,
				0,
				true,
				color_bit_resolution_,
				color_coding_type_,
				timeStamp,
				do_voxel_centroid_enDecoding_
				);
		intra_coder.defineBoundingBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
		intra_coder.encodePointCloud(intra_coded_points, i_coded_data);

		//compute the convergence statistics
		shared_macroblock_percentage_ = (float)shared_macroblock_count / (float)macro_block_count;
		shared_macroblock_convergence_percentage_ = (float)convergence_count / (float)shared_macroblock_count;

		// clean up
		delete i_block_tree;
		delete p_block_tree;
	}
   /** \brief routine to decode a delta frame (predictive coding)
    * \author Rufael Mekuria rufael.mekuria@cwi.nl
    * \param icloud_arg  point cloud to be used a I frame
    * \param cloud_out_arg  decoded frame
    * \param i_coded_data intra encoded data 
    * \param p_coded_data inter encoded data
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> bool
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::decodePointCloudDeltaFrame( 
      const PointCloudConstPtr &icloud_arg, PointCloudPtr &cloud_out_arg, 
      std::istream& i_coded_data, std::istream& p_coded_data)
    {
      ////////////// generate the macroblocks of the iframe //////////////////////////////
      MacroBlockTree * i_block_tree = generate_macroblock_tree(icloud_arg);

      ///////////// inputs for decoding chunks of p data /////////////////////////////////
      int16_t keys_in[3]={0,0,0};
      uint8_t chunk_size=0;
      char rgb_offsets[3]={0,0,0};
      std::vector<int16_t> comp_dat_in;
      int decoded_mblocks=0;

      ///////////////////////////decode p data ///////////////////////////
      while(p_coded_data.good())
      {
        // read the chunk size first
        p_coded_data.read((char *) &chunk_size, sizeof(uint8_t));

        if((chunk_size > 0) && p_coded_data.good()){

          // read the keys 
          p_coded_data.read((char *) keys_in, 3*sizeof(int16_t));

          // compute size of comp_dat (13 for vector mode a, 6 for quaternion mode)
          uint8_t comp_dat_size = chunk_size - 3*sizeof(int16_t) - (do_icp_color_offset_ ? 3:0);
          comp_dat_in.resize(comp_dat_size/sizeof(int16_t));

          //  load compdat
          p_coded_data.read((char *) &comp_dat_in[0] ,comp_dat_in.size()*sizeof(int16_t));

          //load color offsets
          if(do_icp_color_offset_)
            p_coded_data.read((char *) &rgb_offsets[0] ,3*sizeof(char));

          pcl::octree::OctreeContainerPointIndices *i_leaf;

          if((i_leaf = i_block_tree->findLeaf(keys_in[0],keys_in[1],keys_in[2])))
          {
            typename pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>(*icloud_arg, i_leaf->getPointIndicesVector()));

            // following code is for generation of predicted frame
            Eigen::Matrix4f mdec = Eigen::Matrix4f::Identity();
            Eigen::Quaternion<float> l_quat_out_dec;
            RigidTransformCoding<float>::deCompressRigidTransform(comp_dat_in, mdec,l_quat_out_dec);

            // predicted point cloud
            pcl::PointCloud<PointT> p_chunk;

            // generate the pcoded data from the previous frame
            transformPointCloud<PointT, float>
              (  
              *cloud_in,
              p_chunk, 
              mdec
              );
            decoded_mblocks++;

            //decode the predictive points
            for(int j=0; j<p_chunk.size();j++)
            {
              PointT &p_point = (p_chunk)[j];

              if(do_icp_color_offset_)
              {
                p_point.r+=p_point.r + rgb_offsets[0];
                p_point.g+=p_point.g + rgb_offsets[1];
                p_point.b+=p_point.b + rgb_offsets[2];
              }

              cloud_out_arg->push_back(p_point);
            } 
          }
          else{
            std::cout << " failed decoding predictive frame, no corresponding i block " << std::endl;
          }
        }
        else
          break;
      }
      std::cout << " decoded: " << decoded_mblocks 
        << " pblocks, resulting in " 
        << cloud_out_arg->size() 
        << " output points " 
        << std::endl;

      // decode the intra coded points

      typename pcl::PointCloud<PointT>::Ptr intra_coded_points(new pcl::PointCloud<PointT>());

      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT> intra_coder 
        (
        MANUAL_CONFIGURATION,
        false,
        point_resolution_,
        octree_resolution_,
        true,
        0,
        true,
        color_bit_resolution_,
        color_coding_type_,
			timeStamp,
        do_voxel_centroid_enDecoding_
        );
	  uint64_t t = 0;
        if (!intra_coder.decodePointCloud(i_coded_data,intra_coded_points,t)) return false;

      // add the decoded input points to the output cloud
      for(int i=0; i< intra_coded_points->size();i++)
        cloud_out_arg->push_back((*intra_coded_points)[i]);

      // clean up
      delete i_block_tree;
        return true;
    }

    // 
    struct local_color_enh{
      unsigned int r; 
      unsigned int g;
      unsigned int b; 
    };
    /* EXP *
    //! function for coding an enhancement layer, use the same input cloud!! 
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void 
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::encodeEnhancementLayer(const PointCloudConstPtr &cloud_arg, 
      std::ostream& compressed_tree_data_out_arg)
    {
      // get the leafs
      using namespace octree;
      LeafNodeIterator leaf_it = leaf_begin();
      LeafNodeIterator leaf_end_it = leaf_end();

      StaticRangeCoder color_coder;
      StaticRangeCoder position_coder;

      // temporary storage of the occupancy codes
      std::vector<char> occupancy_codes;
      occupancy_codes.reserve(this->leaf_count_);

      // write the color code back
      std::vector<char> color_vector;

      // iterate the leafs
      while(leaf_it != leaf_end_it)
      {
        // get the point indices
        std::vector<int>& point_indices = leaf_it.getLeafContainer().getPointIndicesVector();

        // get the voxel grid size
        Eigen::Vector3f minpt;
        Eigen::Vector3f maxpt;
        getVoxelBounds(leaf_it, minpt,maxpt);
        
        // middle vertices
        Eigen::Vector3f midden = (maxpt - minpt) * 0.5 + minpt;

        // initialize the code position
        char code_pos=0u;

        // global and local color averages for the color
        unsigned int color_r=0;
        unsigned int color_g=0; 
        unsigned int color_b=0;

        // local color averages for the colors
        std::array<local_color_enh,8> colors_per_cell;
        std::array<int,8> occupancy_count={};

        // iterate all points and compute the occupancy code
        for(int i=0; i < point_indices.size(); i++)
        {
          unsigned char occ_code=0u;
          const PointT &l_point = (*cloud_arg)[point_indices[i]];

          // 
          if(l_point.x > midden.x())
            occ_code+=1;
          
          //
          if(l_point.y > midden.y())
            occ_code+=2;

          //
          if(l_point.z > midden.z())
            occ_code+=4;

          code_pos |= (1u << occ_code);

          // global color average //
          color_r+=l_point.r;
          color_g+=l_point.g;
          color_b+=l_point.b;

          // local color average //
          colors_per_cell[occ_code].r+= l_point.r;
          colors_per_cell[occ_code].g+= l_point.g;
          colors_per_cell[occ_code].b+= l_point.b;
          occupancy_count[occ_code]++;
        }

        // write the occupancy code back
        occupancy_codes.push_back( (char) code_pos);

        color_r/=point_indices.size();
        color_g/=point_indices.size();
        color_b/=point_indices.size();

        ///////////  local color averages /////////////////////
        for(int k=0;k<8;k++)
        {
           if(occupancy_count[k])
           {
             colors_per_cell[k].r/= occupancy_count[k];
             colors_per_cell[k].g/= occupancy_count[k];
             colors_per_cell[k].b/= occupancy_count[k];

             int color_diff_r = (int) color_r - (int) colors_per_cell[k].r;

             if(color_diff_r < 128 && color_diff_r  >= -128)
               color_vector.push_back((char) color_diff_r);
             else
               color_vector.push_back(0);

             int color_diff_g = (int) color_g - (int) colors_per_cell[k].g;
             
             if(color_diff_g < 128 && color_diff_g  >= -128)
               color_vector.push_back((char) color_diff_g);
             else
               color_vector.push_back(0);

             int color_diff_b = (int) color_b - (int) colors_per_cell[k].b;
             
             if(color_diff_b < 128 && color_diff_b  >= -128)
               color_vector.push_back((char) color_diff_b);
             else
               color_vector.push_back(0);
           }
        }
        
        // increment 
        ++leaf_it;
      }
      
      /// format of the enhancement layer  //////
      std::size_t pos_vec_size = occupancy_codes.size();
      std::size_t color_vec_size = color_vector.size();
      std::size_t comp_dat_size=0;

      compressed_tree_data_out_arg.write( (const char *) &pos_vec_size, sizeof(pos_vec_size ));
      compressed_tree_data_out_arg.write( (const char *) &color_vec_size , sizeof(color_vec_size));
   
      comp_dat_size+=position_coder.encodeCharVectorToStream(occupancy_codes, compressed_tree_data_out_arg);
      comp_dat_size+=color_coder.encodeCharVectorToStream(color_vector, compressed_tree_data_out_arg);
    }

    //! function for decoding an enhancment layer
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::decodeEnhancementLayer(std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg, PointCloudPtr &cloud_arg_enh)
    {      
      // decode all the occupancy codes
      std::size_t size_of_colors; 
      std::size_t size_of_occupancy_codes;

      std::vector<char> occ_codes; 
      std::vector<char> color_deltas; 

      compressed_tree_data_in_arg.read((char *) &size_of_colors, sizeof(size_of_colors));
      compressed_tree_data_in_arg.read((char *) &size_of_occupancy_codes,sizeof(size_of_occupancy_codes));

      occ_codes.resize(size_of_occupancy_codes);
      color_deltas.resize(size_of_colors);

      StaticRangeCoder pos_decoder;
      StaticRangeCoder color_decoder;

      pos_decoder.decodeStreamToCharVector(compressed_tree_data_in_arg, occ_codes);
      color_decoder.decodeStreamToCharVector(compressed_tree_data_in_arg, color_deltas);

      // get the leafs
      LeafNodeIterator leaf_it = leaf_begin();
      LeafNodeIterator leaf_end_it = leaf_end();

      std::vector<char>::iterator curr_occ = occ_codes.begin();
      std::vector<char>::iterator color_it= color_deltas.begin();
     
      assert(occ_codes.size() == this->leaf_count_);

      // apply all inverse operations
      // iterate the leafs
      while(leaf_it != leaf_end_it)
      {
        // do the decoding 
        OctreeKey ckey;
        ckey = leaf_it.getCurrentOctreeKey();
        
        for(unsigned char k=0;k<8;k++)
        {
          unsigned char ccode = (1u << k); 
          
          // test occupancy
          if( (*curr_code & ccode) == ccode){
            uint8_t x=0,y=0,z=0;
            PointT newPoint;
            
            int l = k;
            
            if(l >= 4)
            {
              //zpos
              z++;
              l-=4;
            }
            if(l>=2)
            {
               y++;
               l-=2; 
            }
            if(l>=1)
            {
               x++;
            }
            
            //
            newPoint.x = static_cast<float> ((static_cast<double> (ckey.x) + (x ? 0.75 : 0.25)) * resolution_ + min_x_);
            newPoint.y = static_cast<float> ((static_cast<double> (ckey.y) + (y ? 0.75 : 0.25)) * resolution_ + min_y_);
            newPoint.z = static_cast<float> ((static_cast<double> (ckey.z) + (z ? 0.75 : 0.25)) * resolution_ + min_z_);
            
            std::vector<int>& indicesV =leaf_it.getLeafContainer().getPointIndicesVector();
            int index_point = indicesV.size() > 0 ? indicesV[0] : -1;
           
            if(index_point != -1){
              newPoint.r = *color_it++ + (*cloud_arg[index_point]).r;
              newPoint.g = *color_it++ + (*cloud_arg[index_point]).g;
              newPoint.b = *color_it++ + (*cloud_arg[index_point]).b;
            }
            //enhancement layer code decoded
            cloud_arg_enh->push_back(newPoint);           
          }
        }

        // increment iterator
        ++leaf_it;
        ++curr_code;
      }
    }
     EXP */

    /////////////////// CODE OVERWRITING CODEC V1 to become codec V2 ////////////////////////////////

    //! write Frame header, extended for cloud codecV2
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::writeFrameHeader(std::ostream& compressed_tree_data_out_arg)
    {
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (frame_header_identifier_), strlen (frame_header_identifier_));
      //! use the base class and write some extended information on codecV2
	  /*
	  int octree_bits;
	  int color_bits;
	  int jpeg_quality;
	  int macroblock_size;
	  */
      OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::writeFrameHeader(compressed_tree_data_out_arg);

      //! write additional fields for cloud codec v2
	  //std::cout << "\n Input time stamp is: " << timeStamp << "\n";
	  compressed_tree_data_out_arg.write(reinterpret_cast<const char*> (&timeStamp), sizeof(timeStamp));
	  compressed_tree_data_out_arg.write(reinterpret_cast<const char*> (&octreeResolution), sizeof(octreeResolution));
	  compressed_tree_data_out_arg.write(reinterpret_cast<const char*> (&colorBitResolution), sizeof(colorBitResolution));
	  compressed_tree_data_out_arg.write(reinterpret_cast<const char*> (&jpeg_quality), sizeof(jpeg_quality));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&do_voxel_centroid_enDecoding_), sizeof (do_voxel_centroid_enDecoding_)); // centroid coding
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&do_connectivity_encoding_), sizeof (do_connectivity_encoding_));        // connectivity coding (not yet added)
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&create_scalable_bitstream_), sizeof (create_scalable_bitstream_));     // scalable bitstream
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&color_coding_type_), sizeof (color_coding_type_));                    // color coding type (added jpeg based modes)
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&macroblock_size_), sizeof (macroblock_size_));                       // settings for ICP based prediction
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&do_icp_color_offset_), sizeof (do_icp_color_offset_));              // settings for ICP based prediction
    };

    //! read Frame header, extended for cloud codecV2
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::readFrameHeader(std::istream& compressed_tree_data_in_arg)
    {
      //! use the base class and read some extended information on codecV2
      OctreePointCloudCompression<PointT>::readFrameHeader(compressed_tree_data_in_arg);

      //! read additional fields for cloud codec v2
	  compressed_tree_data_in_arg.read(reinterpret_cast<char*> (&timeStamp), sizeof(timeStamp));
	  compressed_tree_data_in_arg.read(reinterpret_cast<char*> (&octreeResolution), sizeof(octreeResolution));
	  compressed_tree_data_in_arg.read(reinterpret_cast<char*> (&colorBitResolution), sizeof(colorBitResolution));
	  compressed_tree_data_in_arg.read(reinterpret_cast<char*> (&jpeg_quality), sizeof(jpeg_quality));
	  //std::cout << "\n Timestamp is :" << timeStamp;
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&do_voxel_centroid_enDecoding_), sizeof (do_voxel_centroid_enDecoding_));
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&do_connectivity_encoding_), sizeof (do_connectivity_encoding_));
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&create_scalable_bitstream_), sizeof (create_scalable_bitstream_));
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&color_coding_type_), sizeof (color_coding_type_));
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&macroblock_size_), sizeof (macroblock_size_));
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&do_icp_color_offset_), sizeof (do_icp_color_offset_));
    };

    /** \brief Encode leaf node information during serialization, added jpeg color coding and voxel centroid coding
     * \param leaf_arg: reference to new leaf node
     * \param key_arg: octree key of new leaf node
     * \brief added centroids encoding compared to the original codec of PCL
     */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::serializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg)
    {
      // reference to point indices vector stored within octree leaf //
      const std::vector<int>& leafIdx = leaf_arg.getPointIndicesVector();
      double lowerVoxelCorner[3];
      PointT centroid;
      // //

      // calculate lower voxel corner based on octree key //
      lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * resolution_ + min_x_;
      lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * resolution_ + min_y_;
      lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * resolution_ + min_z_;
      // //

      // points detail encoding
      if (!do_voxel_grid_enDecoding_)
      {
        // encode amount of points within voxel
        point_count_data_vector_.push_back (static_cast<int> (leafIdx.size ()));

        // differentially encode points to lower voxel corner
        point_coder_.encodePoints (leafIdx, lowerVoxelCorner, input_);

        if (cloud_with_color_) {
          // encode color of points
          if(!color_coding_type_) {
            color_coder_.encodePoints (leafIdx, point_color_offset_, input_);
          } else {
            jp_color_coder_.encodePoints (leafIdx, point_color_offset_, input_);
          }
        }
      }
      else // centroid or voxelgrid encoding
      {
        if (cloud_with_color_)
        {
          // encode average color of all points within voxel
          if(!color_coding_type_)
            color_coder_.encodeAverageOfPoints (leafIdx, point_color_offset_, input_);
          else
            jp_color_coder_.encodeAverageOfPoints (leafIdx, point_color_offset_, input_);
          std::vector<char> & l_colors = color_coding_type_ ? jp_color_coder_.getAverageDataVectorB() : color_coder_.getAverageDataVector();

          //! get the colors from the vector from the color coder, use to store to do temporal prediction
          centroid.r =  l_colors[l_colors.size() - 1];
          centroid.g =  l_colors[l_colors.size() - 2];
          centroid.b =  l_colors[l_colors.size() - 3];
        }
        if(!do_voxel_centroid_enDecoding_)
        {
          centroid.x = lowerVoxelCorner[0] + 0.5 * resolution_ ;
          centroid.y = lowerVoxelCorner[1] + 0.5 * resolution_ ;
          centroid.z = lowerVoxelCorner[2] + 0.5 * resolution_ ;
        }
        else
        {
          Eigen::Vector4f f_centroid;
          pcl::compute3DCentroid<PointT>(*(input_), leafIdx, f_centroid);

          centroid.x = f_centroid[0];
          centroid.y = f_centroid[1];
          centroid.z = f_centroid[2];

          centroid_coder_.encodePoint(lowerVoxelCorner, centroid);
        }
        // store the simplified cloud so that it can be used for predictive encoding
        output_->points.push_back(centroid);
      }
    }

    /** \brief Decode leaf nodes information during deserialization, added jpeg and voxel centroid coding
     * \param key_arg octree key of new leaf node
     * \brief added centroids encoding compared to the original codec
     */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::deserializeTreeCallback (LeafT&, const OctreeKey& key_arg)
    {
      double lowerVoxelCorner[3];
      std::size_t pointCount, i, cloudSize;
      PointT newPoint;
      pointCount = 1;

      if (!do_voxel_grid_enDecoding_)
      {
        // get current cloud size
        cloudSize = output_->points.size();

        // get amount of point to be decoded
        pointCount = *point_count_data_vector_iterator_;
        point_count_data_vector_iterator_++;

        // increase point cloud by amount of voxel points
        for (i = 0; i < pointCount; i++)
          output_->points.push_back (newPoint);

        // calculcate position of lower voxel corner
        lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * resolution_ + min_x_;
        lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * resolution_ + min_y_;
        lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * resolution_ + min_z_;

        // decode differentially encoded points
        point_coder_.decodePoints (output_, lowerVoxelCorner, cloudSize, cloudSize + pointCount);
      }
      else
      {
        // decode the centroid or the voxel center
        if(do_voxel_centroid_enDecoding_)
        {
          PointT centroid_point;

          // calculcate position of lower voxel corner
          lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * resolution_ + min_x_;
          lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * resolution_ + min_y_;
          lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * resolution_ + min_z_;

          centroid_coder_.decodePoint(newPoint,lowerVoxelCorner);
        }
        else
        {
          // calculate center of lower voxel corner
          newPoint.x = static_cast<float> ((static_cast<double> (key_arg.x) + 0.5) * resolution_ + min_x_);
          newPoint.y = static_cast<float> ((static_cast<double> (key_arg.y) + 0.5) * resolution_ + min_y_);
          newPoint.z = static_cast<float> ((static_cast<double> (key_arg.z) + 0.5) * resolution_ + min_z_);
        }

        // add point to point cloud
        output_->points.push_back (newPoint);
      }

      if (cloud_with_color_)
      {
        if (data_with_color_)
          // decode color information
          if(!color_coding_type_)
            color_coder_.decodePoints (output_, output_->points.size () - pointCount,
            output_->points.size (), point_color_offset_);
          else
            jp_color_coder_.decodePoints (output_, output_->points.size () - pointCount, output_->points.size (), point_color_offset_);
        else
          // set default color information

          color_coder_.setDefaultColor(output_, output_->points.size() - pointCount, output_->points.size(), point_color_offset_);
      }
    }

    /** \brief Synchronize to frame header
      * \param compressed_tree_data_in_arg: binary input stream
      * \brief use the new v2 header
      */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> bool
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::syncToHeader (std::istream& compressed_tree_data_in_arg)
    {
      // sync to frame header
		//std::cout << " \n Entered sync to frame header\n";
      unsigned int header_id_pos = 0;
	  //std::cout << "\n Frame header identifier is :" << frame_header_identifier_;
      while (header_id_pos < strlen (frame_header_identifier_))
      {
        char readChar;
        if (compressed_tree_data_in_arg.eof()) return false;
        compressed_tree_data_in_arg.read (static_cast<char*> (&readChar), sizeof (readChar));
        if (readChar != frame_header_identifier_[header_id_pos++])
          header_id_pos = (frame_header_identifier_[0]==readChar)?1:0;
		//std::cout << "\n Header_id_pos is: " << header_id_pos;
		//std::cout << "\n Read char is:" << readChar << "   Current frame header char is :" << frame_header_identifier_[header_id_pos];
      }
	  //std::cout << " \n Left loop\n";
      //! read the original octree header
      OctreePointCloudCompression<PointT>::syncToHeader (compressed_tree_data_in_arg);
      return true;
    };

    /** \brief Apply entropy encoding to encoded information and output to binary stream, added bitstream scalability and centroid encoding compared to  V1
      * \param compressed_tree_data_out_arg1 binary voxel output stream
      * \param compressed_tree_data_out_arg2 binary color output stream
      */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::entropyEncoding (std::ostream& compressed_tree_data_out_arg1, std::ostream& compressed_tree_data_out_arg2 )
    {
      uint64_t binary_tree_data_vector_size;
      uint64_t point_avg_color_data_vector_size;

      compressed_point_data_len_ = 0;
      compressed_color_data_len_ = 0;

      // encode binary octree structure
      binary_tree_data_vector_size = binary_tree_data_vector_.size ();
      compressed_tree_data_out_arg1.write (reinterpret_cast<const char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
      compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream(binary_tree_data_vector_, compressed_tree_data_out_arg1);

      // store the number of bytes used for voxel encoding
      compression_performance_metrics[0] = compressed_point_data_len_ ;

      // encode centroids
      if(do_voxel_centroid_enDecoding_)
      {
        // encode differential centroid information
        std::vector<char>& point_diff_data_vector = centroid_coder_.getDifferentialDataVector ();
        uint32_t point_diff_data_vector_size = (uint32_t) point_diff_data_vector.size();
        compressed_tree_data_out_arg1.write (reinterpret_cast<const char*> (&point_diff_data_vector_size), sizeof (uint32_t));
        compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream(point_diff_data_vector, compressed_tree_data_out_arg1);
      }

      // store the number of bytes used for voxel centroid encoding
      compression_performance_metrics[1] = compressed_point_data_len_ - compression_performance_metrics[0];

      // encode colors
      if (cloud_with_color_)
      {
        // encode averaged voxel color information
        std::vector<char>& pointAvgColorDataVector = color_coding_type_ ?  jp_color_coder_.getAverageDataVector () : color_coder_.getAverageDataVector ();
        point_avg_color_data_vector_size = pointAvgColorDataVector.size ();
        compressed_tree_data_out_arg1.write (reinterpret_cast<const char*> (&point_avg_color_data_vector_size), sizeof (point_avg_color_data_vector_size));
        compressed_color_data_len_ += entropy_coder_.encodeCharVectorToStream(pointAvgColorDataVector, compressed_tree_data_out_arg1);
      }

      // store the number of bytes used for the color stream
      compression_performance_metrics[2] = compressed_color_data_len_ ;

      // flush output stream
      compressed_tree_data_out_arg1.flush ();

      if (!do_voxel_grid_enDecoding_)
      {
        uint64_t pointCountDataVector_size;
        uint64_t point_diff_data_vector_size;
        uint64_t point_diff_color_data_vector_size;

        // encode amount of points per voxel
        pointCountDataVector_size = point_count_data_vector_.size();
        compressed_tree_data_out_arg2.write (reinterpret_cast<const char*> (&pointCountDataVector_size), 
          sizeof (pointCountDataVector_size));
        compressed_point_data_len_ += entropy_coder_.encodeIntVectorToStream(point_count_data_vector_,
          compressed_tree_data_out_arg2);

        // encode differential point information
        std::vector<char>& point_diff_data_vector = point_coder_.getDifferentialDataVector();
        point_diff_data_vector_size = point_diff_data_vector.size ();
        compressed_tree_data_out_arg2.write (reinterpret_cast<const char*> (&point_diff_data_vector_size), sizeof (point_diff_data_vector_size));
        compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream(point_diff_data_vector, compressed_tree_data_out_arg2);

        if (cloud_with_color_)
        {
          // encode differential color information
          std::vector<char>& point_diff_color_data_vector = color_coding_type_ ?  jp_color_coder_.getDifferentialDataVector () : color_coder_.getDifferentialDataVector ();
          point_diff_color_data_vector_size = point_diff_color_data_vector.size ();
          compressed_tree_data_out_arg2.write (reinterpret_cast<const char*> (&point_diff_color_data_vector_size),
            sizeof (point_diff_color_data_vector_size));
          compressed_color_data_len_ += entropy_coder_.encodeCharVectorToStream (point_diff_color_data_vector,
            compressed_tree_data_out_arg2);
        }
      }
      // flush output stream
      compressed_tree_data_out_arg2.flush ();
    };

    /** \brief Entropy decoding of input binary stream and output to information vectors, added bitstream scalability and centroid encoding compared to  V1
	    * \param compressed_tree_data_in_arg1 binary voxel input stream
	    * \param compressed_tree_data_in_arg2 binary color enhancement input stream
	    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::entropyDecoding (std::istream& compressed_tree_data_in_arg1, std::istream& compressed_tree_data_in_arg2)
    {
      uint64_t binary_tree_data_vector_size;
      uint64_t point_avg_color_data_vector_size;

      compressed_point_data_len_ = 0;
      compressed_color_data_len_ = 0;

      // decode binary octree structure
      compressed_tree_data_in_arg1.read (reinterpret_cast<char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
      binary_tree_data_vector_.resize(static_cast<std::size_t> (binary_tree_data_vector_size));
      compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector(compressed_tree_data_in_arg1, binary_tree_data_vector_);

      //! new option for encoding centroids
      if(do_voxel_centroid_enDecoding_)
      {
        uint32_t l_count;

        // decode differential point information
        std::vector<char>& pointDiffDataVector = centroid_coder_.getDifferentialDataVector ();
        compressed_tree_data_in_arg1.read (reinterpret_cast<char*> (&l_count), sizeof (uint32_t));
        pointDiffDataVector.resize (static_cast<std::size_t> (l_count));
        compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector(compressed_tree_data_in_arg1, pointDiffDataVector);
      }

      if (data_with_color_)
      {
        // decode averaged voxel color information
        std::vector<char>& point_avg_color_data_vector = color_coding_type_ ? jp_color_coder_.getAverageDataVector() : color_coder_.getAverageDataVector();
        compressed_tree_data_in_arg1.read (reinterpret_cast<char*> (&point_avg_color_data_vector_size), sizeof (point_avg_color_data_vector_size));
        point_avg_color_data_vector.resize (static_cast<std::size_t> (point_avg_color_data_vector_size));
        compressed_color_data_len_ += entropy_coder_.decodeStreamToCharVector(compressed_tree_data_in_arg1,	point_avg_color_data_vector);
      }
/* no enhancement layer encoding/decoding TBD */
      //! check if the enhancement layer has been received
      compressed_tree_data_in_arg2.peek();
      if(compressed_tree_data_in_arg2.good() && (! compressed_tree_data_in_arg2.eof()) )
        do_voxel_grid_enDecoding_ = false;
      else
        do_voxel_grid_enDecoding_ = true;

      if (!do_voxel_grid_enDecoding_)
      {
        uint64_t point_count_data_vector_size;
        uint64_t point_diff_data_vector_size;
        uint64_t point_diff_color_data_vector_size;

        // decode amount of points per voxel
        compressed_tree_data_in_arg2.read (reinterpret_cast<char*> (&point_count_data_vector_size), sizeof (point_count_data_vector_size));
        point_count_data_vector_.resize (static_cast<std::size_t> (point_count_data_vector_size));
        compressed_point_data_len_ += entropy_coder_.decodeStreamToIntVector(compressed_tree_data_in_arg2, point_count_data_vector_);
        point_count_data_vector_iterator_ = point_count_data_vector_.begin ();

        // decode differential point information
        std::vector<char>& pointDiffDataVector = point_coder_.getDifferentialDataVector();
        compressed_tree_data_in_arg2.read (reinterpret_cast<char*> (&point_diff_data_vector_size), sizeof (point_diff_data_vector_size));
        pointDiffDataVector.resize (static_cast<std::size_t> (point_diff_data_vector_size));
        compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg2, pointDiffDataVector);
        if (data_with_color_)
        {
          // decode differential color information
          std::vector<char>& pointDiffColorDataVector = color_coder_.getDifferentialDataVector ();
          compressed_tree_data_in_arg2.read (reinterpret_cast<char*> (&point_diff_color_data_vector_size), sizeof (point_diff_color_data_vector_size));
          pointDiffColorDataVector.resize (static_cast<std::size_t> (point_diff_color_data_vector_size));
          compressed_color_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg2, pointDiffColorDataVector);
        }
      }
      /* enhancement layer encoding/decoding EXP */
    };
    /** \brief
      *  \param point_clouds: an array of pointers to point_clouds to be inspected and modified
      * by \ref pcl_outlier_filter.
      */
    template<typename PointT,  typename LeafT, typename BranchT, typename OctreeT> void OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::remove_outliers (vector<PointCloudPtr> &point_clouds, int min_points, double radius, unsigned int debug_level)
    {
      // apply a radius filter to remove outliers
      typedef OctreePointCloudCompression<PointXYZRGB> colorOctreeCodec;
      if(min_points > 0)
      {
        for(int i=0;i<point_clouds.size();i++){
          colorOctreeCodec::PointCloudPtr l_ptr= colorOctreeCodec::PointCloudPtr(new colorOctreeCodec::PointCloud());
          pcl::RadiusOutlierRemoval<PointT> rorfilter (true); // Initializing with true will allow us to  extract the removed indices
          rorfilter.setInputCloud (point_clouds[i]);
          rorfilter.setRadiusSearch (radius);
          rorfilter.setMinNeighborsInRadius (min_points);
          rorfilter.setNegative (false);
          rorfilter.filter (*l_ptr);
        
          // swap the pointer
          IndicesConstPtr indices_rem = rorfilter.getRemovedIndices ();
          // point_§clouds[i]->erase(indices_rem->begin(),indices_rem->end());
          // The resulting cloud_out contains all points of cloud_in that have 'min_points' or more neighbors within the 'radius' search area
          point_clouds[i] = l_ptr;
          if (debug_level > 2) {
            std::cout << "filtered out a total of: " << indices_rem->size() << " outliers" <<std::endl;
          }
          // The indices_rem array indexes all points of cloud_in that have less than 'min_points' neighbors  within the 0.1 'radius' search area
        }
      }
    }
    /** \brief
      *  \param point_clouds: a vector of pointers to point_clouds to be inspected and modified
      * to normalize their bouding boxes s.t. they effectivly can be used for interframe coding.
      */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> BoundingBox OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::normalize_pointclouds(vector<PointCloudPtr> &point_clouds, vector<BoundingBox, Eigen::aligned_allocator<BoundingBox> > &bounding_boxes, double bb_expand_factor, vector<float> dynamic_range, vector<float> offset, unsigned int debug_level)
    {
      Eigen::Vector4f min_pt_bb(0,0,0,0);
      Eigen::Vector4f max_pt_bb;
      pcl::io::BoundingBox bb;
        
      bool is_bb_init = false;
      int bb_align_count=0;
      std::vector<bool> aligned_flags(point_clouds.size()); // flags to check if a cloud is aligned
      
      // initial bounding box
      min_pt_bb[0]= 1000;
      min_pt_bb[1]= 1000;
      min_pt_bb[2]= 1000;
      
      max_pt_bb[0]= -1000;
      max_pt_bb[1]= -1000;
      max_pt_bb[2]= -1000;
      
      for(int k=0;k<point_clouds.size();k++){
	    Eigen::Vector4f min_pt;
        Eigen::Vector4f max_pt;
        
        pcl::getMinMax3D<pcl::PointXYZRGB>(*point_clouds[k],min_pt,max_pt);
          
        if (debug_level > 3) cerr << "[ " << min_pt.x() << "," << min_pt.y() << "," << min_pt.z() << "]    [" << max_pt.x() << "," << max_pt.y() << "," << max_pt.z() <<"]" << endl;
        
        // check if min fits bounding box, otherwise adapt the bounding box
        if( !( (min_pt.x() > min_pt_bb.x()) && (min_pt.y() > min_pt_bb.y()) && (min_pt.z() > min_pt_bb.z())))
        {
          is_bb_init = false;
        }
        
        // check if max fits bounding box, otherwise adapt the bounding box
        if(!((max_pt.x() < max_pt_bb.x()) && (max_pt.y() < max_pt_bb.y()) && (max_pt.z() < max_pt_bb.z())))
        {
          is_bb_init = false;
        }
        
        if(!is_bb_init)
        {
          aligned_flags[k] = false;
          bb_align_count++;
          // initialize the bounding box, with bb_expand_factor extra
          min_pt_bb[0] = min_pt[0] - bb_expand_factor*abs(max_pt[0] - min_pt[0]);
          min_pt_bb[1] = min_pt[1] - bb_expand_factor*abs(max_pt[1] - min_pt[1]);
          min_pt_bb[2] = min_pt[2] - bb_expand_factor*abs(max_pt[2] - min_pt[2]);
          
          max_pt_bb[0] = max_pt[0] + bb_expand_factor*abs(max_pt[0] - min_pt[0]);
          max_pt_bb[1] = max_pt[1] + bb_expand_factor*abs(max_pt[1] - min_pt[1]);
          max_pt_bb[2] = max_pt[2] + bb_expand_factor*abs(max_pt[2] - min_pt[2]);
          
          is_bb_init = true;
          
          cout << "re-intialized bounding box !!! " << endl;
        }
        else
          aligned_flags[k] = true;
        
        Eigen::Vector4f  dyn_range = max_pt_bb - min_pt_bb;
        
        bounding_boxes[k].max_xyz = max_pt_bb;
        bounding_boxes[k].min_xyz = min_pt_bb;
        
        for(int j=0; j < point_clouds[k]->size();j++)
        {
          // offset the minimum value
          point_clouds[k]->at(j).x-=min_pt_bb[0];
          point_clouds[k]->at(j).y-=min_pt_bb[1];
          point_clouds[k]->at(j).z-=min_pt_bb[2];
          
          // dynamic range
          point_clouds[k]->at(j).x/=dyn_range[0];
          point_clouds[k]->at(j).y/=dyn_range[1];
          point_clouds[k]->at(j).z/=dyn_range[2];
        }
        
        // bounding box is expanded
        
        Eigen::Vector4f min_pt_res;
        Eigen::Vector4f max_pt_res;
        
        pcl::getMinMax3D<pcl::PointXYZRGB>(*(point_clouds[k]),min_pt_res,max_pt_res);
        
        assert(min_pt_res[0] >= 0);
        assert(min_pt_res[1] >= 0);
        assert(min_pt_res[2] >= 0);
        
        assert(max_pt_res[0] <= 1);
        assert(max_pt_res[1] <= 1);
        assert(max_pt_res[2] <= 1);
      }
      bb.min_xyz = min_pt_bb;
      bb.max_xyz = max_pt_bb;
        
      return bb;
    }
      
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::restore_scaling (PointCloudPtr &point_cloud, const BoundingBox& bb)
    {
      Eigen::Vector4f  dyn_range = bb.max_xyz - bb.min_xyz;
      dyn_range[3] = 1; // avoid uninitialized variables
        
      if (point_cloud->size() > 0)
        for (int l = 0; l < point_cloud->size(); l++)
        {
		  // dynamic range
		  point_cloud->at(l).x *= dyn_range[0];
		  point_cloud->at(l).y *= dyn_range[1];
		  point_cloud->at(l).z *= dyn_range[2];
		  // offset the minimum value
          point_cloud->at(l).x += bb.min_xyz[0];
          point_cloud->at(l).y += bb.min_xyz[1];
          point_cloud->at(l).z += bb.min_xyz[2];
         }
    }
  }
}
#endif
