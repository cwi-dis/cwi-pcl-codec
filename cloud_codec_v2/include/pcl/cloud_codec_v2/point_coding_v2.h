/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2011-2012, Willow Garage, Inc.
*  Copyright (c) 2015- Centrum Wiskunde en Informatica.
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

#ifndef POINT_COMPRESSIONV2_H
#define POINT_COMPRESSIONV2_H

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

namespace pcl
{
  namespace octree
  {
    /** \brief @b PointCodingV2 class, extends the initial point coding class with single point encoding
    * \note This class encodes 8-bit differential point information for octree-based point cloud compression.
    * \note typename: PointT: type of point used in pointcloud
    * \author Rufael Mekuria rufael.mekuria@cwi.nl
    */
    template<typename PointT>
    class PointCodingV2 : public PointCoding<PointT>
    {
    public:
      // public typedefs
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef boost::shared_ptr<PointCloud> PointCloudPtr;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      /** \brief Constructor. */
      PointCodingV2 () : PointCoding<PointT>()
      {
      }

      /** \brief Empty class constructor. */
      virtual
        ~PointCodingV2 ()
      {
      }

      /** \brief Encode differential point information for a single point
      * The result is stored in the differnetial point vector
      * \param referencePoint_arg coordinates of reference point
      * \param idxPoint input point
      */
      void
      encodePoint (const double* referencePoint_arg, const PointT &idxPoint)
      {
        unsigned char diffX, diffY, diffZ;

        // differentially encode point coordinates and truncate overflow
        diffX = static_cast<unsigned char> (max (-127, min<int>(127, static_cast<int> ((idxPoint.x - referencePoint_arg[0])  / this->pointCompressionResolution_))));
        diffY = static_cast<unsigned char> (max (-127, min<int>(127, static_cast<int> ((idxPoint.y - referencePoint_arg[1])  / this->pointCompressionResolution_))));
        diffZ = static_cast<unsigned char> (max (-127, min<int>(127, static_cast<int> ((idxPoint.z - referencePoint_arg[2])  / this->pointCompressionResolution_))));

        // store information in differential point vector
        this->pointDiffDataVector_.push_back (diffX);
        this->pointDiffDataVector_.push_back (diffY);
        this->pointDiffDataVector_.push_back (diffZ);
      }

      /** \brief Decode differential point information
      * \param outputPoint_arg output point
      * \param referencePoint_arg coordinates of reference point
      */
      void
      decodePoint (PointT &outputPoint_arg, const double* referencePoint_arg)
      {
        // retrieve differential point information
        const unsigned char& diffX = static_cast<unsigned char> (*(this->pointDiffDataVectorIterator_++));
        const unsigned char& diffY = static_cast<unsigned char> (*(this->pointDiffDataVectorIterator_++));
        const unsigned char& diffZ = static_cast<unsigned char> (*(this->pointDiffDataVectorIterator_++));

        // retrieve point from point cloud
        PointT& point = outputPoint_arg;

        // decode point position
        point.x = static_cast<float> (referencePoint_arg[0] + diffX * this->pointCompressionResolution_);
        point.y = static_cast<float> (referencePoint_arg[1] + diffY * this->pointCompressionResolution_);
        point.z = static_cast<float> (referencePoint_arg[2] + diffZ * this->pointCompressionResolution_);
      }
    };
  }
}

#define PCL_INSTANTIATE_ColorCodingV2(T) template class PCL_EXPORTS pcl::octree::ColorCoding<T>;

#endif
