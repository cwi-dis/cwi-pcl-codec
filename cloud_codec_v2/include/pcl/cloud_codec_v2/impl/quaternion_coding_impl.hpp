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

// implementation of quaternion encoding based on 

/*
* Compresses the quaternion from traditional 4*4 = 16 bytes to 6 bytes.
* This is a lossy compression based on article "Quaternion, Compression"
* by Zarb-Adami in GPG3.
*/
#ifndef QUATERNION_CODING_CCV2_HPP
#define QUATERNION_CODING_CCV2_HPP

#include <pcl/cloud_codec_v2/quaternion_coding.h>

namespace pcl{

  namespace io
  {

    bool QuaternionCoding::compressQuaternion(Eigen::Quaternion<float> &quat_in, int16_t *comp_dat)
    {
      static const float scale = 1.41421f;

      float &x = quat_in.x();
      float &w = quat_in.w();
      float &y = quat_in.y();
      float &z = quat_in.z();

      int16_t &s0 = comp_dat[0];
      int16_t &s1 = comp_dat[1];
      int16_t &s2 = comp_dat[2];

      if (w > x && w > y && w > z) {

        float rx = x * scale;
        float ry = y * scale;
        float rz = z * scale;

        // w is the biggest element, make sure it's also positive, if not, negate the quaternion
        if (w < 0) {
          rx = -rx;
          ry = -ry;
          rz = -rz;
        }

        if (rx < -1) rx = -1;
        else if (rx > 1) rx = 1;
        if (ry < -1) ry = -1;
        else if (ry > 1) ry = 1;
        if (rz < -1) rz = -1;
        else if (rz > 1) rz = 1;

        s0 = int16_t(rx * 32767);
        s1 = int16_t((int(ry * 32767) & 0xfffe) | 1);
        s2 = int16_t((int(rz * 32767) & 0xfffe) | 1);
      }
      else if (z > x && z > y) {

        float rx = x * scale;
        float ry = y * scale;
        float rw = w * scale;

        // z is the biggest element, make sure it's also positive, if not, negate the quaternion
        if (z < 0) {
          rx = -rx;
          ry = -ry;
          rw = -rw;
        }

        if (rx < -1) rx = -1;
        else if (rx > 1) rx = 1;
        if (ry < -1) ry = -1;
        else if (ry > 1) ry = 1;
        if (rw < -1) rw = -1;
        else if (rw > 1) rw = 1;

        s0 = int16_t(rx * 32767);
        s1 = int16_t((int(ry * 32767) & 0xfffe) | 1);
        s2 = int16_t((int(rw * 32767) & 0xfffe) | 0);
      }
      else if (y > x) {

        float rx = x * scale;
        float rz = z * scale;
        float rw = w * scale;

        // y is the biggest element, make sure it's also positive, if not, negate the quaternion
        if (y < 0) {
          rx = -rx;
          rz = -rz;
          rw = -rw;
        }

        if (rx < -1) rx = -1;
        else if (rx > 1) rx = 1;
        if (rz < -1) rz = -1;
        else if (rz > 1) rz = 1;
        if (rw < -1) rw = -1;
        else if (rw > 1) rw = 1;

        s0 = int16_t(rx * 32767);
        s1 = int16_t((int(rz * 32767) & 0xfffe) | 0);
        s2 = int16_t((int(rw * 32767) & 0xfffe) | 1);
      }
      else {

        float ry = y * scale;
        float rz = z * scale;
        float rw = w * scale;

        // x is the biggest element, make sure it's also positive, if not, negate the quaternion
        if (x < 0) {
          ry = -ry;
          rz = -rz;
          rw = -rw;
        }

        if (ry < -1) ry = -1;
        else if (ry > 1) ry = 1;
        if (rz < -1) rz = -1;
        else if (rz > 1) rz = 1;
        if (rw < -1) rw = -1;
        else if (rw > 1) rw = 1;

        s0 = int16_t(ry * 32767);
        s1 = int16_t((int(rz * 32767) & 0xfffe) | 0);
        s2 = int16_t((int(rw * 32767) & 0xfffe) | 0);
      }

      return true;
    };

    bool QuaternionCoding::deCompressQuaternion(int16_t *comp_dat, Eigen::Quaternion<float> &quat_out)
    {
      float &x = quat_out.x();
      float &w = quat_out.w();
      float &y = quat_out.y();
      float &z = quat_out.z();

      int16_t &s0 = comp_dat[0];
      int16_t &s1 = comp_dat[1];
      int16_t &s2 = comp_dat[2];

      int which = ((s1 & 1) << 1) | (s2 & 1);
      s1 &= 0xfffe;
      s2 &= 0xfffe;

      static const float scale = 1.0f / 32767.0f / 1.41421f;

      if (which == 3) {
        x = s0 * scale;
        y = s1 * scale;
        z = s2 * scale;

        w = 1 - (x*x) - (y*y) - (z*z);
        if (w > FLT_EPSILON)
          w = sqrt(w);
      }
      else if (which == 2) {
        x = s0 * scale;
        y = s1 * scale;
        w = s2 * scale;

        z = 1 - (x*x) - (y*y) - (w*w);
        if (z > FLT_EPSILON)
          z = sqrt(z);
      }
      else if (which == 1) {
        x = s0 * scale;
        z = s1 * scale;
        w = s2 * scale;

        y = 1 - (x*x) - (z*z) - (w*w);
        if (y > FLT_EPSILON)
          y = sqrt(y);
      }
      else {
        y = s0 * scale;
        z = s1 * scale;
        w = s2 * scale;

        x = 1 - (y*y) - (z*z) - (w*w);
        if (x > FLT_EPSILON)
          x = sqrt(x);
      }
      return true;
    };
  
  }
}
#endif