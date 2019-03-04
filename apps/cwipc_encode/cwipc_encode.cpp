#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include <cwipc_codec/api.h>

#define WITH_BOOST_SHARED_POINTER

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << "pointcloudfile.ply compressedfile.cwicpc" << std::endl;
        return 2;
    }
    //
    // Read pointcloud file
    //
    pcl::PointCloud<pcl::PointXYZRGB> *pc = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PLYReader ply_reader;
    if (ply_reader.read(argv[1], *pc) < 0) {
        std::cerr << argv[0] << ": Error reading pointcloud from " << argv[1] << std::endl;
        return 1;
    }
    std::cerr << "Read pointcloud successfully, " << pc->size() << " points." << std::endl;
    //
    // Compress
    //
    encoder_params param;
	param.num_threads = 1;
	param.do_inter_frame = false;
	param.gop_size = 1;
	param.exp_factor = 0;
	param.octree_bits = 7;
	param.color_bits = 8;
	param.jpeg_quality = 85;
	param.macroblock_size = 16;
    cwi_encode encoder;
    std::stringstream outputBuffer;
#ifdef WITH_BOOST_SHARED_POINTER
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pcBoost(pc);
    void *pcVoidPtr = reinterpret_cast<void*>(&pcBoost);
#else
    void *pcVoidPtr = reinterpret_cast<void*>(pc);
#endif
//    boost::shared_ptr<pcl::PointCloud<PointT> > pointcloud = *reinterpret_cast<boost::shared_ptr<pcl::PointCloud<PointT> >*>(pc);
    
    if (encoder.cwi_encoder(param, pcVoidPtr, outputBuffer, 0) < 0) {
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

