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

#include <pcl/point_cloud.h>
#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>

using namespace std;

class cwipc_encoder_impl : public cwipc_encoder
{
public:
    cwipc_encoder_impl(cwipc_encoder_params *_params)
    :   m_params(*_params),
        m_result(NULL),
        m_result_size(0)
    {}
    ~cwipc_encoder_impl() {}
    void free() {
        if (m_result) ::free(m_result);
        m_result = NULL;
        m_result_size = 0;
    };
    bool eof() { return false; };
    bool available(bool wait) { return m_result != NULL; };
    void feed(cwipc *pc) {
        std::stringstream comp_frame;
        double point_resolution = std::pow ( 2.0, -1.0 * m_params.octree_bits);
        double octree_resolution = std::pow ( 2.0, -1.0 * m_params.octree_bits);
        boost::shared_ptr<pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> > encoder_V2_;
        encoder_V2_ = boost::shared_ptr<pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> > (
            new pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> (
                  pcl::io::MANUAL_CONFIGURATION,
                  false,
                  point_resolution,
                  octree_resolution,
                  true, // no intra voxel coding in this first version of the codec
                  0, // i_frame_rate,
                  m_params.color_bits > 0 ? true : false,
                  m_params.color_bits,
                  1,
                  pc->timestamp(),
                  false,
                  false, // not implemented
                  false, // do_connectivity_coding_ not implemented
                  m_params.jpeg_quality,
                  m_params.num_threads
                  ));
        encoder_V2_->setMacroblockSize(m_params.macroblock_size);
        cwipc_pcl_pointcloud pcl_pc = pc->access_pcl_pointcloud();
        encoder_V2_->encodePointCloud(pcl_pc, comp_frame);
        /* xxxjack should lock here */
        if (m_result) {
            ::free(m_result);
            m_result = NULL;
            m_result_size = 0;
        }
        m_result_size = comp_frame.str().length();
        m_result = (void *)malloc(m_result_size);
        comp_frame.str().copy((char *)m_result, m_result_size);
    };
    size_t get_encoded_size() { return m_result_size; };
    bool copy_data(void *buffer, size_t bufferSize) {
        if (m_result == NULL || bufferSize < m_result_size) return false;
        memcpy(buffer, m_result, m_result_size);
        ::free(m_result);
        m_result = NULL;
        m_result_size = 0;
        return true;
    };
private:
    cwipc_encoder_params m_params;
    void *m_result;
    size_t m_result_size;
};

class _CWIPC_CODEC_EXPORT cwipc_decoder_impl : public cwipc_decoder
{
public:
    cwipc_decoder_impl() 
    : m_result(NULL)
    {}
    ~cwipc_decoder_impl() {}
    void free() {};
    bool eof() {return false; };
    bool available(bool wait) { return m_result != NULL; };
    void feed(void *buffer, size_t bufferSize) {
        cwipc_encoder_params par;
        //Default codec parameter values set in signals
        par.num_threads = 1;
        par.do_inter_frame = false;
        par.gop_size = 1;
        par.exp_factor = 0;
        par.octree_bits = 7;
        par.color_bits = 8;
        par.jpeg_quality = 85;
        par.macroblock_size = 16;
        std::stringstream compfr;
        //Convert buffer to stringstream for encoding
        std::string str((char *)buffer, bufferSize);
        std::stringstream istream(str);
        boost::shared_ptr<pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> > decoder_V2_;
        decoder_V2_ = boost::shared_ptr<pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> > (
            new pcl::io::OctreePointCloudCodecV2<cwipc_pcl_point> (
                pcl::io::MANUAL_CONFIGURATION,
                false,
                std::pow ( 2.0, -1.0 * par.octree_bits ),
                std::pow ( 2.0, -1.0 * par.octree_bits),
                true, // no intra voxel coding in this first version of the codec
                0, // i_frame_rate,
                true,
                par.color_bits,
                1,
                0,
                false,
                false, // not implemented
                false, // do_connectivity_coding_, not implemented
                par.jpeg_quality,
                par.num_threads
                ));
        cwipc_pcl_pointcloud decpc = new_cwipc_pcl_pointcloud();
        uint64_t tmStmp = 0;
        bool ok = decoder_V2_->decodePointCloud(istream, decpc, tmStmp);
        if (ok) {
            m_result = cwipc_from_pcl(decpc, tmStmp, NULL);
        } else {
            m_result = NULL;
        }
    };
    cwipc *get() {
        cwipc *rv = m_result;
        m_result = NULL;
        return rv;
    };
private:
    cwipc *m_result;
};

cwipc_encoder* cwipc_new_encoder(int version, cwipc_encoder_params *params) {
    if (version != CWIPC_ENCODER_PARAM_VERSION) {
        return NULL;
    }
    if (params == NULL) {
    	static cwipc_encoder_params dft = { 1, false, 1, 1.0, 8, 8, 85, 16};
    	params = &dft;
	}
    return new cwipc_encoder_impl(params);
}

void cwipc_encoder_free(cwipc_encoder *obj) {
    obj->free();
}

bool cwipc_encoder_available(cwipc_encoder *obj, bool wait) {
    return obj->available(wait);
}

bool cwipc_encoder_eof(cwipc_encoder *obj) {
    return obj->eof();
}

void cwipc_encoder_feed(cwipc_encoder *obj, cwipc* pc) {
    obj->feed(pc);
}

size_t cwipc_encoder_get_encoded_size(cwipc_encoder *obj) {
    return obj->get_encoded_size();
}

bool cwipc_encoder_copy_data(cwipc_encoder *obj, void *buffer, size_t bufferSize) {
    return obj->copy_data(buffer, bufferSize);
}

cwipc_decoder* cwipc_new_decoder() {
    return new cwipc_decoder_impl();
}

void cwipc_decoder_feed(cwipc_decoder *obj, void *buffer, size_t bufferSize) {
    obj->feed(buffer, bufferSize);
}
