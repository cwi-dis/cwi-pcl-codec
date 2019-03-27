#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "cwipc_util/api_pcl.h"
#include "cwipc_codec/api.h"

int main(int argc, char** argv)
{
	uint64_t timestamp = 0LL;
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << "pointcloudfile.ply compressedfile.cwicpc [timestamp]" << std::endl;
        return 2;
    }
    if (argc > 3) {
    	timestamp = (uint64_t)atoll(argv[3]);
    }
    //
    // Read pointcloud file
    //
    cwipc_pcl_pointcloud pc = new_cwipc_pcl_pointcloud();
    pcl::PLYReader ply_reader;
    if (ply_reader.read(argv[1], *pc) < 0) {
        std::cerr << argv[0] << ": Error reading pointcloud from " << argv[1] << std::endl;
        return 1;
    }
    std::cerr << "Read pointcloud successfully, " << pc->size() << " points." << std::endl;
    //
    // Compress
    //
    cwipc_encoder_params param;
	param.num_threads = 1;
	param.do_inter_frame = false;
	param.gop_size = 1;
	param.exp_factor = 0;
	param.octree_bits = 7;
	param.color_bits = 8;
	param.jpeg_quality = 85;
	param.macroblock_size = 16;
    cwipc_codec encoder(param);
    std::stringstream outputBuffer;
//    boost::shared_ptr<pcl::PointCloud<PointT> > pointcloud = *reinterpret_cast<boost::shared_ptr<pcl::PointCloud<PointT> >*>(pc);
    
    if (encoder.compress_to_stream(pc, outputBuffer, timestamp) < 0) {
        std::cerr << argv[0] << ": Error encoding pointcloud from " << argv[1] << std::endl;
        return 1;
    }
    std::cerr << "Encoded successfully, " << outputBuffer.tellp() << " bytes." << std::endl;
    //
    // Save to output
    //
    std::ofstream output(argv[2]);
    output << outputBuffer.rdbuf();
    output.close();
    return 0;
}

