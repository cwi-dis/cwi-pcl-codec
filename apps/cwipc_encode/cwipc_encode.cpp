#include <iostream>
#include <fstream>
#include <unistd.h>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include <cwipc_codec/api.h>

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << "pointcloudfile.ply compressedfile.cwicpc" << std::endl;
        return 2;
    }
    char cwdbuf[200];
    std::cerr << "cwd:" << getcwd(cwdbuf, sizeof(cwdbuf)) << std::endl;
    //
    // Read pointcloud file
    //
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::PLYReader ply_reader;
    pcl::PolygonMesh mesh;
    if (ply_reader.read(argv[1], pc) < 0) {
        std::cerr << argv[0] << ": Error reading pointcloud from " << argv[1] << std::endl;
        return 1;
    }
    std::cerr << "Read pointcloud successfully, " << pc.size() << " points." << std::endl;
//    pcl::fromPCLPointCloud2(mesh.cloud, pc);
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
    void *pcVoidPtr = reinterpret_cast<void*>(&pc);
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

#if 0
nt
load_ply_file (std::string path, boost::shared_ptr<pcl::PointCloud<PointT> > pc)
{
    int rv = 1;
    PLYReader ply_reader;
    /* next straighforward code crashes, work around via PolygonMesh *
     PCLPointCloud2 pc2;
     if (rv && ply_reader.read (path, pc2) < 0)
     {
     fromPCLPointCloud2 (pc2, *pc);
     rv = 0;
     }
     */
    PolygonMesh mesh;
    if (rv && ply_reader.read (path, mesh) >= 0) {
        pcl::fromPCLPointCloud2 (mesh.cloud, *pc);
    } else {
        rv= 0;
    }
    return rv;
}
#endif
