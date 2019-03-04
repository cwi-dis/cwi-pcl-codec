// cwi_encode.cpp : Defines the exported functions for the DLL application.
//
#include <cstdint>
#include <chrono>
#include <sstream>
#ifdef WIN32
#define _CWIPC_CODEC_EXPORT __declspec(dllexport)
#else
#define _CWIPC_CODEC_EXPORT
#endif

#include "cwipc_util/api_pcl.h"
#include "cwipc_codec/api.h"
#include "cwipc_codec/evaluate_comp.h"
#include "evaluate_comp_impl.hpp"


using namespace std;

int cwi_encode::cwi_encoder(cwipc_encoder_params param, cwipc_pcl_pointcloud pc, std::stringstream& comp_frame, std::uint64_t timeStamp)
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

int cwi_encode::cwi_decoder(cwipc_encoder_params param, cwipc_pcl_pointcloud pc, std::stringstream& comp_frame, uint64_t &timeStamp)
{
	evaluate_comp_impl<PointXYZRGB> evaluate;
	return evaluate.evaluate_dc(param, pc, comp_frame, timeStamp) == true ? 0 : -1;
}

//Exporting  a unity compliant decode function and point cloud data structure


//Decode function to receive a compressed point cloud as a c# Byte[] and return a point cloud as a Mypointcloud object
cwipc* cwipc_decompress(unsigned char * compFrame, int len)
{
	cwipc_encoder_params par;
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
	evaluate_comp_impl<cwipc_pcl_point> evaluate;
    cwipc_pcl_pointcloud decpc = new_cwipc_pcl_pointcloud();
	uint64_t tmStmp = 0;
#ifdef DEBUG
	std::ofstream logsize1;
	logsize1.open("log.txt", std::ofstream::app);
	logsize1 << "\n Decoder called";
	logsize1.close();
#endif // DEBUG
	evaluate.evaluate_dc(par, decpc, compfr, tmStmp);
#ifdef DEBUG
	std::ofstream log3;
	log3.open("log.txt", std::ofstream::app);
	log3 << "\n Point cloud extracted size is :" << (*decpc).points.size();
	log3.close();
#endif // DEBUG
    return cwipc_from_pcl(decpc, tmStmp, NULL);
}
