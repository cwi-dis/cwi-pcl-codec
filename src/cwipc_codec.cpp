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

// Some parameters that will eventually be customizable again
const int num_threads = 1;
const bool downdownsampling = false;
const int iframerate = 1;

class cwipc_encoder_impl : public cwipc_encoder
{
public:
    cwipc_encoder_impl(cwipc_encoder_params *_params)
    :   m_params(*_params),
        m_result(NULL),
        m_result_size(0)
    {

	}
    
    ~cwipc_encoder_impl() {}
    void free() {
        if (m_result) ::free(m_result);
        m_result = NULL;
        m_result_size = 0;
    };
    
    bool eof() { return false; };
    
    bool available(bool wait) { return m_result != NULL; };
    
    bool at_gop_boundary() { return true; };
    
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
                  downdownsampling, // no intra voxel coding in this first version of the codec
                  iframerate, // i_frame_rate,
                  true, // do color encoding
                  8, // color bits
                  1, // color coding type
                  pc->timestamp(),
                  false,
                  false, // not implemented
                  false, // do_connectivity_coding_ not implemented
                  m_params.jpeg_quality,
                  num_threads
                  ));
        encoder_V2_->setMacroblockSize(m_params.macroblock_size);
        cwipc_pcl_pointcloud pcl_pc = pc->access_pcl_pointcloud();
        if (pcl_pc->size() == 0) {
        	// Special case: if the point cloud is empty we compress a point cloud with a single black point at 0,0,0
        	pcl_pc = new_cwipc_pcl_pointcloud();
        	cwipc_pcl_point dummyPoint;
        	pcl_pc->push_back(dummyPoint);
        }
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
        par.do_inter_frame = false;
        par.gop_size = 1;
        par.exp_factor = 1.0;
        par.octree_bits = 8;
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
                downdownsampling, // no intra voxel coding in this first version of the codec
                iframerate, // i_frame_rate,
                true,
                8,
                1,
                0,
                false,
                false, // not implemented
                false, // do_connectivity_coding_, not implemented
                par.jpeg_quality,
                num_threads
                ));
        cwipc_pcl_pointcloud decpc = new_cwipc_pcl_pointcloud();
        uint64_t tmStmp = 0;
        bool ok = decoder_V2_->decodePointCloud(istream, decpc, tmStmp);
        if (decpc->size() == 1) {
            // Special case: single point (0,0,0,0,0,0) signals an empty pointcloud
            cwipc_pcl_point& pt(decpc->at(0));
            if (abs(pt.x) < 0.01 && abs(pt.y) < 0.01 && abs(pt.z) < 0.01
                && pt.r < 2 && pt.g < 2 && pt.b < 2
                ) {
                decpc->clear();
            }
        }
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

cwipc_encoder* cwipc_new_encoder(int version, cwipc_encoder_params *params, char **errorMessage) {
    if (version != CWIPC_ENCODER_PARAM_VERSION) {
    	*errorMessage = (char *)"cwpic_bew_encoder: incorrect encoder param version";
        return NULL;
    }
    if (params == NULL) {
    	static cwipc_encoder_params dft = {false, 1, 1.0, 9, 85, 16, 0, 0};
    	params = &dft;
	}
	// Check parameters for this release
	if (params->do_inter_frame) {
		*errorMessage = (char *)"cwipc_new_encoder: do_inter_frame must be false for this version";
		return NULL;
	}
	if (params->gop_size != 1) {
		*errorMessage = (char *)"cwipc_new_encoder: gop_size must be 1 for this version";
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

bool cwipc_encoder_at_gop_boundary(cwipc_encoder *obj) {
    return obj->at_gop_boundary();
}


cwipc_decoder* cwipc_new_decoder() {
    return new cwipc_decoder_impl();
}

void cwipc_decoder_feed(cwipc_decoder *obj, void *buffer, size_t bufferSize) {
    obj->feed(buffer, bufferSize);
}
