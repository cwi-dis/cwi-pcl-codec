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
//  evaluate_compression.h
//  evaluate_compression
//
//  Created by Kees Blom on 01/06/16.
//
//

#ifndef evaluate_comp_h
#define evaluate_comp_h
#include <vector>
#include <string>

class evaluate_comp {
protected:
  //evaluate_comp (int argc, char** argv) : argc_(argc), argv_(argv), output_index_(-1) {}
	evaluate_comp() {}
  private:
    // get_options_from_file
    // get_options_from_commandline
    // load_pcl_file
    // load_pcl_file_in_diroctory
    // initialize
    // remove_outliers
    // normalize_bouding_box
    // encode
    // encode_group
    // decode
    // decode_group
    // get_codec_quality
    // visualize
    //
    // int evaluate(const int argc, const char* argv[]);
    //
  protected: // variables
// input settings
    int argc_;
    char** argv_;
    std::vector<std::string> input_directories_;
// output settings
    int write_out_ply_;
    bool show_statistics_ = false;
    bool visualization_ = false;
    std::string output_directory_;
    std::string intra_frame_quality_csv_;
    std::string predictive_quality_csv_;
// program control settings
    int group_size_;
    std::string algorithm_; // select compression algorithm
    int testbbalign_;  // testing the bounding box alignment algorithm ?TBD want to keep this ?
    int debug_level_;
    int num_threads_;
// outlier removal
    int K_outlier_filter_;
    double radius_;
// boundingbox expansion and normalisation
    double bb_expand_factor_;
// compression codec settings
    double point_resolution_;
    double octree_resolution_;
    int enh_bits_;          // enhancement resolution YBD
    int octree_bits_;       // XYZ resoution 
    int color_bits_;        // color resolution
    int color_coding_type_;
    bool keep_centroid_;
    int icp_on_original_;
    int macroblock_size_;
    bool do_icp_color_offset_;
    bool do_delta_coding_;
    bool do_quality_computation_;
    bool do_icp_on_original_;
    bool create_scalable_;
    int jpeg_quality_;
    int output_index_;
	std::uint64_t timeStamp_;
  };
 /* evaluate_compression_h */
struct encoder_params
{
	int num_threads;
	bool do_inter_frame;
	int gop_size;
	double exp_factor;
	int octree_bits;
	int color_bits;
	int jpeg_quality;
	int macroblock_size;
};
#endif