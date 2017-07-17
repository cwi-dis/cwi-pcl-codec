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
#ifndef RIGID_TRANSFORM_CODING_CCV2_HPP
#define RIGID_TRANSFORM_CODING_CCV2_HPP

// class for quaternion encoding
#include <Eigen/Geometry>
#include <stdint.h>

// use quaternions
#include <pcl/cloud_codec_v2/rigid_transform_coding.h>
#include <pcl/cloud_codec_v2/impl/quaternion_coding_impl.hpp>
#include <limits>

namespace pcl{

  namespace io
  {

    /*!
    * \brief  compression of Rigid Transform resulting from ICP procedure (either as quaternion or 2 vectors and sign byte)
    * \author Rufael Mekuria rufael.mekuria@cwi.nl
    * \param  tr_in  input rigid transform
    * \param  comp_dat_out output words containing compressed transform
    * \param  quat_out output quaternion
    * \note The output vector will contain 6 16-bit words if quaternion encoding was used, otherwise 7.
    */

    template <typename Scalar> bool 
      RigidTransformCoding<Scalar>::compressRigidTransform(
        const Eigen::Matrix<Scalar, 4, 4>& tr_in, 
        std::vector<int16_t>& comp_dat_out,
        Eigen::Quaternion<Scalar>& quat_out)
    {
      // 2,5 is the maximum translation
      const float scaling_factor = (float) std::numeric_limits<int16_t>::max()/2.5;
      
      // convert the rotation to a quaternion
#if _WIN32
      Eigen::Matrix<Scalar,3,3> rotm = tr_in.block<3,3>(0,0);
      quat_out = Eigen::Quaternion<Scalar>(tr_in.block<3,3>(0,0));
#else
      Eigen::Matrix<Scalar,3,3> rotm = tr_in.block(0,0,3,3);
      quat_out = Eigen::Quaternion<Scalar>(rotm);
#endif//_WIN32
      // test the quaternion mode for coding the rotation (encode,decode check if stable)
      Eigen::Quaternion<Scalar> quat_test;
      comp_dat_out.resize(3);
      QuaternionCoding::compressQuaternion(quat_out,(int16_t *) &comp_dat_out[0]);
      QuaternionCoding::deCompressQuaternion(&comp_dat_out[0],quat_test);
      Eigen::Matrix<Scalar, 3, 3> res_rot = quat_test.toRotationMatrix();
 
      //std::cout << "rigid tf coding original " << std::endl << tr_in.block<3,3>(0,0);
      //std::cout << "rigid tf test  quaternion " << std::endl << res_rot;

      // check if decoding the matrix from the quaternion gives a stable result
      bool quaternion_is_stable=true;

      for(int i=0; i<9;i++)
      {
        float diff=0.0;
        if((diff=std::abs(res_rot(i/3,i%3) - tr_in(i/3,i%3))) > 0.001){
          quaternion_is_stable=false;
          //std::cout << "instable"<<std::endl;
          break;
        }
      }
      //if(quaternion_is_stable)
      //  std::cout << " stable " <<std::endl;
      
      // compress the rotation matrix as two vectors if quaternion method is not numerically stable
      if(!quaternion_is_stable){
         comp_dat_out.resize(7);
         // only store two vectors of the rotation matrix
         for(int l=0;l<3;l++){
           comp_dat_out[l] =  (int16_t) int(rotm(0,l) * (std::numeric_limits<int16_t>::max() -1));
           comp_dat_out[l+3] =  (int16_t) int(rotm(1,l) * (std::numeric_limits<int16_t>::max() -1));
           
           // add a byte for storing the signs
           comp_dat_out[6]+= rotm(2,l)<0? 1<<l:0;
           std::cout << " stored bits: " <<  int(1<<l) << std::endl;
         }
      }
      else{
        // make sure to code the rotation properly to the outputvector as a quantized quaternion
        QuaternionCoding::compressQuaternion(quat_out,(int16_t *) &comp_dat_out[0]);
      }

      // quantize the translation

      float t1 = (float) tr_in(0,3);
      if(t1>2.5)
        t1=2.5;
      if(t1<-2.5)
        t1=-2.5;

      float t2 = (float) tr_in(1,3);
      if(t2>2.5)
        t2=2.5;
      if(t2<-2.5)
        t2=-2.5;

      float t3 = (float) tr_in(2,3);
      if(t3>2.5)
        t3=2.5;
      if(t3<-2.5)
        t3=-2.5;

      // compress the translation by quantization using 16 bits
      comp_dat_out.push_back( (int16_t) (int) (t1 * (scaling_factor-1)));
      comp_dat_out.push_back( (int16_t) (int) (t2 * (scaling_factor-1)));
      comp_dat_out.push_back( (int16_t)  (int)(t3 * (scaling_factor-1)));

      return true;
    }

    /*!
    * \brief  decompression of Rigid Transform resulting from ICP procedure (either as quaternion or 2 vectors and sign byte)
    * \author Rufael Mekuria rufael.mekuria@cwi.nl
    * \param  comp_dat_in input words containing compressed transform
    * \param  tr_out  output rigid transform
    * \param  quat_out output quaternion (will be only changed when quaternion encoding was used)
	*/
    template <typename Scalar> bool 
      RigidTransformCoding<Scalar>::deCompressRigidTransform(
        const std::vector<int16_t>& comp_dat_in, 
        Eigen::Matrix<Scalar, 4, 4>& tr_out, 
        Eigen::Quaternion<Scalar>& quat_out)
    {
      // 2 is the maximum 
      const float scaling_factor = (float) std::numeric_limits<int16_t>::max()/2.5;
      
      if(comp_dat_in.size() == 6){
        // decode rotation offset which is coded as quaternion
        QuaternionCoding::deCompressQuaternion((int16_t *) &comp_dat_in[0], quat_out);
#if _WIN32
        tr_out.block<3,3>(0,0) = quat_out.toRotationMatrix();
        //std::cout << "decoded rotation" << std::endl << tr_out.block<3,3>(0,0) << std::endl;
#else
          tr_out.block(0,0,3,3) = quat_out.toRotationMatrix();
          //std::cout << "decoded rotation" << std::endl << tr_out.block<3,3>(0,0) << std::endl;
#endif//_WIN323
      }
      else{
        //decode the rotation matrix from two vectors and the sign vectors
         for(int l=0;l<3;l++){
           tr_out(0,l) = ((float) comp_dat_in[l]) / (std::numeric_limits<int16_t>::max() -1);
           tr_out(1,l) = ((float) comp_dat_in[l+3]) / (std::numeric_limits<int16_t>::max() -1);
           tr_out(2,l) =  std::sqrt(1 - tr_out(0,l) * tr_out(0,l) - tr_out(1,l) * tr_out(1,l));
           
           // decode the sign
           if( ((1<<l) & ((int)comp_dat_in[6])) == 1<<l)
             tr_out(2,l)= - tr_out(2,l);
         }
      }

      // decode offset 
      tr_out(0,3) = ((Scalar) comp_dat_in[comp_dat_in.size()-3]) / ((Scalar) (scaling_factor -1)); 
      tr_out(1,3) = ((Scalar) comp_dat_in[comp_dat_in.size()-2]) / ((Scalar) (scaling_factor -1));
      tr_out(2,3) = ((Scalar) comp_dat_in[comp_dat_in.size()-1]) / ((Scalar) (scaling_factor -1));
      
      // construct transform matrix
      tr_out(3,0) = 0;
      tr_out(3,1) = 0;
      tr_out(3,2) = 0;
      tr_out(3,3) = 1;

      return true;
    }
  }
}
#endif