// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the CWI_ENCODE_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// CWI_ENCODE_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
// point cloud library
//#include <pcl/point_types.h>
//#include <pcl/compression/point_coding.h>
//#include <pcl/io/impl/octree_pointcloud_compression.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/compression/octree_pointcloud_compression.h>
//#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>
//#include <pcl/filters/radius_outlier_removal.h>
// This class is exported from the cwi_encode.dll
//class CWI_ENCODE_API Ccwi_encode {
//public:
	//Ccwi_encode(void);
	// TODO: add your methods here.
//};

//extern CWI_ENCODE_API int ncwi_encode;

// Note by xxxjack: this is incorrect, really: the dllexport is needed when building the
// codec library, but it should be a dllimport when using it. To be fixed later.
#ifdef WIN32
#define _CWI_DLL_EXPORT __declspec(dllexport)
#else
#define _CWI_DLL_EXPORT
#endif

//
// Memory allocator that allocates blocks that can be transferred to csharp.
//

#ifdef WIN32
#define CSHARP_COMPAT_ALLOC(nbytes) GlobalAlloc(GPTR, (nbytes))
#else
#define CSHARP_COMPAT_ALLOC(nbytes) malloc(nbytes)
#endif

//CWI_ENCODE_API int fncwi_encode(void);
//CWI_ENCODE_API int cwi_encoder(encoder_params param, void* pc, std::stringstream& comp_frame);
//Unity compliant point cloud data structure
struct MyPoint
{
	float x;
	float y;
	float z;
	int8_t r;
	int8_t g;
	int8_t b;
};

struct MyPointCloud
{
	MyPoint * pointcloud;
	int size;
	uint64_t timeStamp;
};
class _CWI_DLL_EXPORT cwi_encode
{
public:
	int cwi_encoder(encoder_params param, void* pc, std::stringstream& comp_frame, std::uint64_t timeStamp);
	int cwi_decoder(encoder_params param, void* pc, std::stringstream& comp_frame, std::uint64_t &timeStamp);
};
_CWI_DLL_EXPORT int load_ply_file_XYZRGB(std::string path, void **pc);
_CWI_DLL_EXPORT void delete_ply_data(void *pc);

//extern "C" __declspec(dllexport) int cwi_encoder(encoder_params param, void* pc, std::stringstream& comp_frame, std::uint64_t timeStamp);
extern "C" _CWI_DLL_EXPORT MyPointCloud Cwi_decoder(unsigned char * compFrame, int len);
extern "C" _CWI_DLL_EXPORT MyPointCloud Cwi_test2(char* filename, void *p);
