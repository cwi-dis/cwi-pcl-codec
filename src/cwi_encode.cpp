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

int 
cwipc_codec::compress_to_stream(cwipc_pcl_pointcloud pc, std::stringstream& comp_frame, std::uint64_t timeStamp)
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

int 
cwipc_codec::decompress_from_stream(cwipc_pcl_pointcloud pc, std::stringstream& comp_frame, uint64_t &timeStamp)
{
	evaluate_comp_impl<PointXYZRGB> evaluate;
	return evaluate.evaluate_dc(param, pc, comp_frame, timeStamp) == true ? 0 : -1;
}

//Exporting  a unity compliant decode function and point cloud data structure


//Decode function to receive a compressed point cloud as a c# Byte[] and return a point cloud as a Mypointcloud object
cwipc* 
cwipc_decompress(unsigned char * compFrame, int len)
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
