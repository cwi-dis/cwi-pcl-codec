#include <stdint.h>
#ifdef __cplusplus
#include <sstream>
#include <cwipc_codec/evaluate_comp.h>
#endif

// For Windows ensure that the symbols are imported from a DLL, unless we're compiling the DLL itself.
#ifndef _CWIPC_CODEC_EXPORT
#ifdef WIN32
#define _CWIPC_CODEC_EXPORT __declspec(dllimport)
#else
#define _CWIPC_CODEC_EXPORT
#endif
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
typedef struct _MyPoint
{
	float x;
	float y;
	float z;
	int8_t r;
	int8_t g;
	int8_t b;
} MyPoint;

typedef struct _MyPointCloud
{
	MyPoint * pointcloud;
	int size;
	uint64_t timeStamp;
} MyPointCloud;

#ifdef __cplusplus
class _CWIPC_CODEC_EXPORT cwi_encode
{
public:
	int cwi_encoder(encoder_params param, void* pc, std::stringstream& comp_frame, uint64_t timeStamp);
	int cwi_decoder(encoder_params param, void* pc, std::stringstream& comp_frame, uint64_t &timeStamp);
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

_CWIPC_CODEC_EXPORT MyPointCloud Cwi_decoder(unsigned char * compFrame, int len);
#ifdef xxxjack_old
_CWIPC_CODEC_EXPORT MyPointCloud Cwi_test2(char* filename, void *p);
#endif // xxxjack_old
    
#ifdef __cplusplus
}
#endif
