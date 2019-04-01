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

/** \brief Pointcloud encoder settings.
 *
 * Various parameters governing encoder quality and performance settings.
 *
 */
struct cwipc_encoder_params
{
    int num_threads;        /**< How many concurrent threads to use. */
    bool do_inter_frame;	/**< (unimplemented) do inter-frame compression */
    int gop_size;			/**< (unimplemented) spacing of I frames for inter-frame compression */
    double exp_factor;		/**< Bounding box expansion factor */
    int octree_bits;		/**< Octree resolution */
    int color_bits;			/**< Color resolution */
    int jpeg_quality;		/**< JPEG encoding quality */
    int macroblock_size;	/**< JPEG macroblock size */
};

/** \brief Version of cwipc_encoder_params structure.
 *
 * The external representation of the encoder parameters may change between versions of
 * this library. Therefore when you pass the structure to the constructor you should
 * also pass this version number, to ensure your code is compatible with this version
 * of the library.
 */
#define CWIPC_ENCODER_PARAM_VERSION 0x20190330

#ifdef __cplusplus
class _CWIPC_CODEC_EXPORT cwipc_codec
{
public:
	cwipc_codec(cwipc_encoder_params& _param) : param(_param) {}
	int compress_to_stream(cwipc_pcl_pointcloud pc, std::stringstream& comp_frame, uint64_t timeStamp);
	int decompress_from_stream(cwipc_pcl_pointcloud pc, std::stringstream& comp_frame, uint64_t &timeStamp);
protected:
   cwipc_encoder_params param;
};

/** \brief Pointcloud encoder, abstract C++ interface.
 *
 * This interface is provided by pointcloud compressors. The caller
 * feeds in pointclouds (as `cwipc` objects) and it returns memory blocks
 * with compressed pointcloud data.
 */
class _CWIPC_CODEC_EXPORT cwipc_encoder
{
public:
    virtual ~cwipc_encoder() {}

    /** \brief Deallocate the encoder.
     *
     * Because the encoder may be used in a different implementation
     * language or DLL than where it is implemented we do not count on refcounting
     * and such. Call this method if you no longer need it.
     */
    virtual void free() = 0;

    /** \brief Return true if no more encoded buffers are forthcoming.
     */
    virtual bool eof() = 0;

    /** \brief Return true if a data buffer is currently available.
     * \param wait Set to true if the caller is willing to wait until one is available.
     * 
     * If this cwipc_encoder is not multi-threading capable the wait parameter is ignored.
     * If it is multi-threaded aware and no data is currently available 
     * it may wait a reasonable amount of time (think: about a second) to see whether
     * one becomes available.
     */
    virtual bool available(bool wait) = 0;
    
    /** \brief Encode a pointcloud.
     *
     * This call presents the encoder with a new pointcloud to encode.
     */
    virtual void feed(cwipc *pc) = 0;
    
    /** \brief Return size (in bytes) of compressed data available.
     *
     * The caller should first call `available()` to check that compressed
     * data is available, then `get_encoded_size()` to check how much data is available,
     * then allocate a buffer of suitable size, then call `copy_data()`.
     */
    virtual size_t get_encoded_size() = 0;
    
    /** \brief Obtain compressed pointcloud data.
     *
     * Copies the data of the most recently compressed pointcloud into a buffer
     * supplied by the caller.
     * The compressed data is returned at most once, so after this call
     * `available()` will return false again until `feed()` has been called.
     */
    virtual bool copy_data(void *buffer, size_t bufferSize) = 0;
};

/** \brief Pointcloud encoder, abstract C++ interface.
 *
 * This interface is provided by pointcloud decompressors. The caller
 * feeds in memory blocks with compressed pointcloud data. The interface
 * adheres to the `cwipc_source` interface and produces `cwipc` objects on the
 * output side.
 */
class _CWIPC_CODEC_EXPORT cwipc_decoder : public cwipc_source
{
public:
    virtual ~cwipc_decoder() {}
    virtual void free() = 0;
    virtual bool eof() = 0;
    virtual bool available(bool wait) = 0;

    /** \brief Feed compressed data into the decoder.
	 * \param buffer Pointer to the data buffer containing the compressed pointcloud data.
	 * \param bufferSize Size of the data buffer.
     *
     * Use this call to pass a new compressed pointcloud into the decoder.
     * After decompression `available()` will return true, and `get()` can be
     * used to obtain the cwipc pointcloud data.
     */
    virtual void feed(void *buffer, size_t bufferSize) = 0;
    virtual cwipc *get() = 0;
};
#else
typedef struct _cwipc_encoder {
    int _dummy;
} cwipc_encoder;

typedef struct _cwipc_decoder {
    cwipc_source source;
} cwipc_decoder;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Decompress a single pointcloud.
 *
 * (This API should be considered deprecated)
 * C-compatible API call to decompress a single pointcloud.
 * \param compFrame A pointer to a buffer with the compressed pointcloud data.
 * \param len Size of he compFrame buffer (in bytes)
 * \returns a cwipc object representing the pointcloud.
 */
_CWIPC_CODEC_EXPORT cwipc* cwipc_decompress(unsigned char * compFrame, int len);

/** \brief Create pointcloud decompressor.
 *
 * \param version Pass in CWIPC_ENCODER_PARAM_VERSION to ensure runtime compatibility.
 * \param params Pointer to a structure with parameters than govern the encoding process.
 * \return A cwipc_encoder object.
 */
_CWIPC_CODEC_EXPORT cwipc_encoder* cwipc_new_encoder(int version, cwipc_encoder_params* params);

/** \brief Deallocate the encoder (C interface).
 * \param obj The cwipc_encoder object.
 *
 * Because the encoder may be used in a different implementation
 * language or DLL than where it is implemented we do not count on refcounting
 * and such. Call this method if you no longer need it.
 */
_CWIPC_CODEC_EXPORT void cwipc_encoder_free(cwipc_encoder *obj);

/** \brief Return true if no more encoded buffers are forthcoming (C interface).
 * \param obj The cwipc_encoder object.
 */
_CWIPC_CODEC_EXPORT bool cwipc_encoder_eof(cwipc_encoder *obj);

/** \brief Return true if a data buffer is currently available (C interface).
 * \param obj The cwipc_encoder object.
 * \param wait Set to true if the caller is willing to wait until one is available.
 * 
 * If this cwipc_encoder is not multi-threading capable the wait parameter is ignored.
 * If it is multi-threaded aware and no data is currently available 
 * it may wait a reasonable amount of time (think: about a second) to see whether
 * one becomes available.
 */
_CWIPC_CODEC_EXPORT bool cwipc_encoder_available(cwipc_encoder *obj, bool wait);

/** \brief Encode a pointcloud (C interface).
 * \param obj The cwipc_encoder object.
 * \param pc The pointcloud to encode.
 *
 * This call presents the encoder with a new pointcloud to encode.
 */
_CWIPC_CODEC_EXPORT void cwipc_encoder_feed(cwipc_encoder *obj, cwipc* pc);

/** \brief Return size (in bytes) of compressed data available (C interface).
 * \param obj The cwipc_encoder object.
 *
 * The caller should first call `available()` to check that compressed
 * data is available, then `get_encoded_size()` to check how much data is available,
 * then allocate a buffer of suitable size, then call `copy_data()`.
 */
_CWIPC_CODEC_EXPORT size_t cwipc_encoder_get_encoded_size(cwipc_encoder *obj);

/** \brief Obtain compressed pointcloud data (C interface).
 * \param obj The cwipc_encoder object.
 * \param buffer Pointer to the buffer where the compressed data will be stored.
 * \param bufferSize Size of buffer (in bytes)
 *
 * Copies the data of the most recently compressed pointcloud into a buffer
 * supplied by the caller.
 * The compressed data is returned at most once, so after this call
 * `available()` will return false again until `feed()` has been called.
 */
_CWIPC_CODEC_EXPORT bool cwipc_encoder_copy_data(cwipc_encoder *obj, void *buffer, size_t bufferSize);

/** \brief Create pointcloud decompressor.
 * \returns cwipc_decoder object representing the decompressor.
 *
 * After creating the decoder you feed it data using the `feed()` method.
 * Whenever `available()` returns true a new pointcloud can be obtained with
 * `get()`.
 */
_CWIPC_CODEC_EXPORT cwipc_decoder* cwipc_new_decoder();

/* Methods defined in the cwipc_source superclass, use those */
#define cwipc_decoder_free(obj) cwipc_source_free(obj)
#define cwipc_decoder_available(obj, wait) cwipc_source_available(obj, wait)
#define cwipc_decoder_eof(obj) cwipc_source_eof(obj)
#define cwipc_decoder_get(obj) cwipc_source_get(obj)

/** \brief Feed compressed data into the decoder (C interface).
 * \param obj The cwipc_encoder object.
 * \param buffer Pointer to the data buffer containing the compressed pointcloud data.
 * \param bufferSize Size of the data buffer.
 *
 * Use this call to pass a new compressed pointcloud into the decoder.
 * After decompression `available()` will return true, and `get()` can be
 * used to obtain the cwipc pointcloud data.
 */
_CWIPC_CODEC_EXPORT void cwipc_decoder_feed(cwipc_decoder *obj, void *buffer, size_t bufferSize);

#ifdef __cplusplus
}
#endif
#endif /* cwipc_codec_api_h */
