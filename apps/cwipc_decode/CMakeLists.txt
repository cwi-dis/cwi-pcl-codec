
cmake_minimum_required(VERSION 3.10.0)

add_executable(cwipc_encode cwipc_decode.c)
include_directories("../../include")
target_link_libraries(cwipc_decode cwipc_codec)

install(TARGETS cwipc_decode RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_decode COMMAND cwipc_decode ${CMAKE_SOURCE_DIR}/tests/fixtures/encoded/pcl_frame1.cwicpc ./pcl_frame1.ply )
