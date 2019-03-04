#ifndef cwipc_codec_api_h
#define cwipc_codec_api_h
#include <stdint.h>
#include "cwipc_util/api.h"
#ifdef __cplusplus
#include <sstream>
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

#ifdef __cplusplus
//
// Parameters to give to the encoder
//
struct cwipc_encoder_params
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

class _CWIPC_CODEC_EXPORT cwi_encode
{
public:
	int cwi_encoder(cwipc_encoder_params param, void* pc, std::stringstream& comp_frame, uint64_t timeStamp);
	int cwi_decoder(cwipc_encoder_params param, void* pc, std::stringstream& comp_frame, uint64_t &timeStamp);
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

_CWIPC_CODEC_EXPORT cwipc* cwipc_decompress(unsigned char * compFrame, int len);
    
#ifdef __cplusplus
}
#endif
#endif /* cwipc_codec_api_h */
