
cmake_minimum_required(VERSION 3.10.0)

add_executable(cwipc_encode cwipc_encode.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_encode cwipc_codec)

install(TARGETS cwipc_encode RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_encode COMMAND cwipc_encode ${CMAKE_SOURCE_DIR}/tests/fixtures/input/pcl_frame1.ply ./pcl_frame1.cwicpc )
