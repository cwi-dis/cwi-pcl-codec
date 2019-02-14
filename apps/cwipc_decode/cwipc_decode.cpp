#include <iostream>
#include <fstream>
#include <cwipc_codec/api.h>

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << "compressedfile.cwicpc pointcloudfile.ply" << std::endl;
        return 2;
    }
    std::ifstream input(argv[1]);
    std::ofstream output(argv[2]);
    input.close();
    output << "Hello world" << std::endl;
    output.close();
    return 0;
}
