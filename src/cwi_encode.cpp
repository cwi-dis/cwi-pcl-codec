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
        evaluate_comp_impl<PointXYZRGB> evaluate;
        bool enc;
        cwipc_pcl_pointcloud pcl_pc = pc->access_pcl_pointcloud();
        enc = evaluate.evaluator(m_params, pcl_pc, comp_frame, pc->timestamp());
        /* xxxjack should lock here */
        if (m_result) {
            ::free(m_result);
            m_result = NULL;
            m_result_size = 0;
        }
        if (enc) {
            m_result_size = comp_frame.str().length();
            m_result = (void *)malloc(m_result_size);
            comp_frame.str().copy((char *)m_result, m_result_size);
        }
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

        evaluate_comp_impl<cwipc_pcl_point> evaluate;
        cwipc_pcl_pointcloud decpc = new_cwipc_pcl_pointcloud();
        uint64_t tmStmp = 0;
        evaluate.evaluate_dc(par, decpc, istream, tmStmp);
        m_result = cwipc_from_pcl(decpc, tmStmp, NULL);
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
