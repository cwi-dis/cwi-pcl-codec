#include <iostream>
#include <fstream>

#include "cwipc_codec/api.h"

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << "compressedfile.cwicpc pointcloudfile.ply" << std::endl;
        return 2;
    }
    //
    // Read compressed file
    //
    std::ifstream input(argv[1]);
    // Determine data size and allocate a buffer
    input.seekg(0, std::ios::end);
    size_t filesize = input.tellg();
    char *inputBuffer = (char *)malloc(filesize);
    if (inputBuffer == NULL) {
    	std::cerr << argv[0] << ": could not allocate " << filesize << " bytes." << std::endl;
    	return 1;
    }
    // Read all data
    input.seekg(0, std::ios::beg);
    input.read(inputBuffer, filesize);
    input.close();
	std::cerr << "Read " << filesize << " compressed bytes." << std::endl;
    //
    // Uncompress
    //
    cwipc_decoder *decoder = cwipc_new_decoder();
    if (decoder == NULL) {
    	std::cerr << argv[0] << ": Could not create decoder" << std::endl;
    	return 1;
    }
    decoder->feed(inputBuffer, filesize);
    free((void *)inputBuffer); // After feed() we can free the input buffer
    bool ok = decoder->available(true);
    if (!ok) {
    	std::cerr << argv[0] << ": Decoder did not create pointcloud" << std::endl;
    	return 1;
    }
    cwipc *pc = decoder->get();
    if (pc == NULL) {
    	std::cerr << argv[0] << ": Decoder did not return cwipc" << std::endl;
    	return 1;
    }
    decoder->free(); // We don't need the encoder anymore
	std::cerr << "Decoded successfully, " <<pc->get_uncompressed_size(CWIPC_POINT_VERSION) << " bytes (uncompressed)" << std::endl;
    //
    // Save pointcloud file
    //
    if (cwipc_write(argv[2], pc, NULL) < 0) {
    	std::cerr << argv[0] << ": Error writing PLY file " << argv[2] << std::endl;
    	return 1;
    }
	pc->free(); // We no longer need to pointcloud
    return 0;
}
