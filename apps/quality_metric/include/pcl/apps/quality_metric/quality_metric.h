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
 *   * Neither the name of copyright holder(s)  nor the names of its
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
//      .h
//  quality_metric
//
//  Created by Kees Blom on 20/08/18.
//
//

#ifndef quality_metric_H
#define quality_metric_H
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/quality_metric/quality_metric.h>

class quality_metric {
  protected:
    quality_metric (int argc, char** argv) : argc_(argc), argv_(argv),  cloud_a_ (""), cloud_b_(""), quality_method_(NONE) {}
  private:
    // get_options_from_commandline
    // load_pcl_files
    // load_pcl_file_in_diroctory (not implemented yet
    // initialize
    //
    // compute quality metric using selected method
  protected: // variables
// input settings
    int argc_;
    char** argv_;
    
//Arguments parsed:
//... input settings
    std::string cloud_a_;
    std::string cloud_b_;
    std::string method_;
    QualityMethod quality_method_;
//... output settings
//X TBD
//... program control settings
    int debug_level_;
    int num_threads_;
  };
#endif /* quality_metric_H */
