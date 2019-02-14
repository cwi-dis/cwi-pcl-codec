// cwi_encode.cpp : Defines the exported functions for the DLL application.
//
#include <cstdint>
#include <chrono>
#include <sstream>
#include "cwipc_codec/evaluate_comp.h"
#include "evaluate_comp_impl.hpp"
#ifdef WIN32
#define _CWIPC_CODEC_EXPORT __declspec(dllexport)
#else
#define _CWIPC_CODEC_EXPORT
#endif

#include "cwipc_codec/api.h"

using namespace std;

int cwi_encode::cwi_encoder(encoder_params param, void* pc, std::stringstream& comp_frame, std::uint64_t timeStamp)
{
	std::uint64_t codecStart;
	codecStart = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
#ifdef DEBUG
	std::cout << "Entered codec at :" << codecStart << " ms" << "\nTime since capture :" << codecStart - timeStamp << " ms";
#endif // DEBUG
	evaluate_comp_impl<PointXYZRGB> evaluate;
	bool enc;
	enc = evaluate.evaluator(param, pc, comp_frame, timeStamp);
	std::uint64_t codecEnd;
	codecEnd = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
#ifdef DEBUG
	std::cout << "Exiting codec at :" << codecEnd << " ms" << ", Encode took " << codecEnd - codecStart << " ms \n";
#endif // DEBUG
	std::cout.flush();
	return enc == true ? 0 : -1;
}

int cwi_encode::cwi_decoder(encoder_params param, void* pc, std::stringstream& comp_frame, uint64_t &timeStamp)
{
	evaluate_comp_impl<PointXYZRGB> evaluate;
	return evaluate.evaluate_dc(param, pc, comp_frame, timeStamp) == true ? 0 : -1;
}

#ifdef xxxjack_old
FILE _iob[] = { *stdin, *stdout, *stderr };
extern "C" FILE * __cdecl __iob_func(void)
{
	return _iob;
}
#endif // xxxjack_old

//Exporting  a unity compliant decode function and point cloud data structure

#ifdef xxxjack_old
//Test function to receive a filename, read a pointcloud .ply and return the contents in a MyPointcloud structure
extern "C" _CWIPC_CODEC_EXPORT MyPointCloud Cwi_test2(char* filename, void *p)
{
	std::string path(filename);
	std::ofstream log1;
	log1.open("log.txt");
	log1 << "\n Entered the function";
	log1.close();
	auto pe = new PLYExport;
	pe->pc = boost::shared_ptr<pcl::PointCloud<PointXYZRGB>>(new PointCloud<PointXYZRGB>());
	std::ofstream log2;
	log2.open("log.txt", std::ofstream::app);
	log2 << "\n Created empty point cloud object :" << path;
	log2.close();
	int res = load_ply_file<PointXYZRGB>(path, pe->pc);
	std::ofstream log3;
	log3.open("log.txt", std::ofstream::app);
	log3 << "\n Load PLY complete size is :" << sizeof(pe->pc);
	log3.close();
	MyPointCloud ptcld;
	pcl::PointCloud<PointXYZRGB> cld = *(pe->pc);
	int size = cld.height * cld.width;
	ptcld.size = size;
	std::ofstream log4;
	log4.open("log.txt", std::ofstream::app);
	log4 << "\n Point count :" << size;
	log4.close();
	ptcld.timeStamp = 0;
	//ptcld.pointcloud = (MyPoint*)::CoTaskMemAlloc(sizeof(MyPoint) * size);
	ptcld.pointcloud = (MyPoint*)CSHARP_COMPAT_ALLOC(sizeof(MyPoint) * size);
	for (int i = 0; i < size; i++)
	{
		(ptcld.pointcloud[i]).x = cld.points[i].x;
		(ptcld.pointcloud[i]).y = cld.points[i].y;
		(ptcld.pointcloud[i]).z = cld.points[i].z;
		(ptcld.pointcloud[i]).r = cld.points[i].r;
		(ptcld.pointcloud[i]).g = cld.points[i].g;
		(ptcld.pointcloud[i]).b = cld.points[i].b;
	}
	std::ofstream log5;
	log4.open("log.txt", std::ofstream::app);
	log4 << "\n Points Assigned with GlobalAlloc";
	log4.close();
	return ptcld;
}
#endif // xxxjack_old

//Decode function to receive a compressed point cloud as a c# Byte[] and return a point cloud as a Mypointcloud object
MyPointCloud Cwi_decoder(unsigned char * compFrame, int len)
{
	encoder_params par;
#ifdef DEBUG
	std::ofstream log1;
	log1.open("log.txt");
	log1 << "\n Decoder Initialised";
	log1.close();
#endif // DEBUG
	//Default codec parameter values set in signals
	par.num_threads = 1;
	par.do_inter_frame = false;
	par.gop_size = 1;
	par.exp_factor = 0;
	par.octree_bits = 7;
	par.color_bits = 8;
	par.jpeg_quality = 85;
	par.macroblock_size = 16;
#ifdef DEBUG
	std::ofstream log2;
	log2.open("log.txt", std::ofstream::app);
	log2 << "\n Codec params set";
	log2.close();
#endif // DEBUG
	std::stringstream compfr;
	//Convert C# bytestream to stringstream for decoding
	for (int i = 0; i < len; i++)
	{
		compfr << compFrame[i];
	}
#ifdef DEBUG
	compfr.seekg(0, ios::end);
	int sizeReceived = compfr.tellg();
	compfr.seekg(0, ios::beg);
	std::ofstream logsize;
	logsize.open("log.txt", std::ofstream::app);
	logsize << "\n Compressed frame of size " << sizeReceived << " received ";
	logsize.close();
#endif // DEBUG
	compfr.seekg(0, ios::beg);
	evaluate_comp_impl<PointXYZRGB> evaluate;
	boost::shared_ptr<pcl::PointCloud<PointXYZRGB> > decpc(new PointCloud<PointXYZRGB>());
	decpc->makeShared();
	void * dpc;
	dpc = reinterpret_cast<void *> (&decpc);
	uint64_t tmStmp = 0;
#ifdef DEBUG
	std::ofstream logsize1;
	logsize1.open("log.txt", std::ofstream::app);
	logsize1 << "\n Decoder called";
	logsize1.close();
#endif // DEBUG
	evaluate.evaluate_dc(par, dpc, compfr, tmStmp);
#ifdef DEBUG
	std::ofstream log3;
	log3.open("log.txt", std::ofstream::app);
	log3 << "\n Point cloud extracted size is :" << (*decpc).points.size();
	log3.close();
#endif // DEBUG
	//Format coversion
	MyPointCloud ptcld;
	pcl::PointCloud<PointXYZRGB> cld = *decpc;
	int size = cld.height * cld.width;
	ptcld.size = size;
	ptcld.pointcloud = (MyPoint*)CSHARP_COMPAT_ALLOC(sizeof(MyPoint) * size);
	ptcld.timeStamp = tmStmp;
	//Store points from PCL pointcloud in MyPointcloud
	for (int i = 0; i < size; i++)
	{
		(ptcld.pointcloud[i]).x = cld.points[i].x;
		(ptcld.pointcloud[i]).y = cld.points[i].y;
		(ptcld.pointcloud[i]).z = cld.points[i].z;
		(ptcld.pointcloud[i]).r = cld.points[i].r;
		(ptcld.pointcloud[i]).g = cld.points[i].g;
		(ptcld.pointcloud[i]).b = cld.points[i].b;
	}
#ifdef DEBUG
	std::ofstream log4;
	log4.open("log.txt", std::ofstream::app);
	log4 << "\n Created MyPointCloud object";
	log4.close();
#endif //DEBUG
	return ptcld;
}
