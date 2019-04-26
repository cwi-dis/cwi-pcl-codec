#include <iostream>
#include <fstream>

#include "cwipc_codec/api.h"

int main(int argc, char** argv)
{
	uint64_t timestamp = 0LL;
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " voxelsize pointcloudfile.ply newpointcloudfile.ply" << std::endl;
        return 2;
    }
	float voxelsize = atof(argv[1]);
    //
    // Read pointcloud file
    //
    char *errorMessage = NULL;
    cwipc *pc = cwipc_read(argv[2], 0LL, &errorMessage);

    if (pc == NULL || errorMessage) {
        std::cerr << argv[0] << ": Error reading pointcloud from " << argv[2] << ": " << errorMessage << std::endl;
        return 1;
    }
    std::cerr << "Read pointcloud successfully, " << pc->get_uncompressed_size(CWIPC_POINT_VERSION) << " bytes (uncompressed)" << std::endl;
    //
    // Voxelize
    //
    cwipc *new_pc = cwipc_downsample(pc, voxelsize);
    //
    // Save pointcloud file
    //
    if (cwipc_write(argv[3], new_pc, NULL) < 0) {
    	std::cerr << argv[0] << ": Error writing PLY file " << argv[3] << std::endl;
    	return 1;
    }
    pc->free();
    new_pc->free();

    return 0;
}

