#include <iostream>
#include <fstream>

#include "cwipc_codec/api.h"

int main(int argc, char** argv)
{
	uint64_t timestamp = 0LL;
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << "pointcloudfile.ply compressedfile.cwicpc [timestamp]" << std::endl;
        return 2;
    }
    if (argc > 3) {
    	timestamp = (uint64_t)atoll(argv[3]);
    }
    //
    // Read pointcloud file
    //
    char *errorMessage = NULL;
    cwipc *pc = cwipc_read(argv[1], 0LL, &errorMessage);

    if (pc == NULL || errorMessage) {
        std::cerr << argv[0] << ": Error reading pointcloud from " << argv[1] << ": " << errorMessage << std::endl;
        return 1;
    }
    std::cerr << "Read pointcloud successfully, " << pc->get_uncompressed_size(CWIPC_POINT_VERSION) << " bytes (uncompressed)" << std::endl;
    //
    // Compress
    //
    cwipc_encoder_params param;
	param.do_inter_frame = false;
	param.gop_size = 1;
	param.exp_factor = 1.0;
	param.octree_bits = 9;
	param.jpeg_quality = 85;
	param.macroblock_size = 16;
	param.tilenumber = 0;

	char *errorString;
    cwipc_encoder *encoder = cwipc_new_encoder(CWIPC_ENCODER_PARAM_VERSION, &param, &errorString);
    if (encoder == NULL) {
    	std::cerr << argv[0] << ": Could not create encoder: " << errorString << std::endl;
    	return 1;
    }
    encoder->feed(pc);
    pc->free();	// After feeding the pointcloud into the encoder we can free it.
    bool ok = encoder->available(true);
    if (!ok) {
    	std::cerr << argv[0] << ": Encoder did not create compressed data" << std::endl;
    	return 1;
    }
    size_t bufSize = encoder->get_encoded_size();
    char *buffer = (char *)malloc(bufSize);
    if (buffer == NULL) {
    	std::cerr << argv[0] << ": Could not allocate " << bufSize << " bytes." << std::endl;
    	return 1;
    }
    ok = encoder->copy_data(buffer, bufSize);
    if (!ok) {
    	std::cerr << argv[0] << ": Encoder could not copy compressed data" << std::endl;
    	return 1;
    }
    encoder->free(); // We don't need the encoder anymore.
    std::cerr << "Encoded successfully, " << bufSize << " bytes." << std::endl;
    //
    // Save to output
    //
    std::ofstream output(argv[2], std::ofstream::binary);
    output << buffer;
    output.close();
    return 0;
}

