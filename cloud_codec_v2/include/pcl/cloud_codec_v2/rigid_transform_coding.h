/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014- Centrum Wiskunde Informatica
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
*   * Neither the name of its copyright holders nor the names of its
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
#ifndef RIGID_TRANSFORM_CODING_CCV2_H
#define RIGID_TRANSFORM_CODING_CCV2_H
// class for rigid transform encoding
#include <Eigen/Geometry>
#include <stdint.h>
#include <vector>

namespace pcl{

  namespace io
  {
     /** \brief @b Rigid Transofrm Coding class
    * \note This class encodes a rigid transform in 12 or 14 bytes (6 or 7 shorts)
    * \author Rufael Mekuria rufael.mekuria@cwi.nl
    */
    template<typename Scalar=float> class RigidTransformCoding{
       
       // static member functions for encoding and decoding
       public:

        static bool 
        compressRigidTransform(const Eigen::Matrix<Scalar, 4, 4> &tr_in, 
                               std::vector<int16_t> &comp_dat_out,
                               Eigen::Quaternion<Scalar> &quat_in);
        
        static bool 
        deCompressRigidTransform(const std::vector<int16_t> &comp_dat_in, 
                                 Eigen::Matrix<Scalar, 4, 4> &tr_out, 
                                 Eigen::Quaternion<Scalar> &quat_out);
    };
  }
}
#endif