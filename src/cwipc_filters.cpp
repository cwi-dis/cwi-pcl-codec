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
#include <pcl/filters/voxel_grid.h>

cwipc *cwipc_downsample(cwipc *pc, float voxelsize)
{
	if (pc == NULL) return NULL;
	cwipc_pcl_pointcloud src = pc->access_pcl_pointcloud();
	if (src == NULL) return NULL;
	cwipc_pcl_pointcloud dst = new_cwipc_pcl_pointcloud();
	// Step 1 - Voxelize
	pcl::VoxelGrid<cwipc_pcl_point> grid;
	grid.setInputCloud(src);
	grid.setLeafSize(voxelsize, voxelsize, voxelsize);
	grid.setSaveLeafLayout(true);
	grid.filter(*dst);
	// Step 2 - Clear tile numbers in destination
	for (auto dstpt : dst->points) {
		dstpt.a = 0;
	}
	// Step 3 - Do OR of all contribution point tile numbers in destination.
	for (auto srcpt : src->points) {
		auto dstIndex = grid.getCentroidIndex(srcpt);
		auto dstpt = dst->points[dstIndex];
		dstpt.a |= srcpt.a;
	}
	return cwipc_from_pcl(dst, pc->timestamp(), NULL);
}

cwipc *cwipc_tilefilter(cwipc *pc, int tile)
{
	if (pc == NULL) return NULL;
	cwipc_pcl_pointcloud src = pc->access_pcl_pointcloud();
	if (src == NULL) return NULL;
	cwipc_pcl_pointcloud dst = new_cwipc_pcl_pointcloud();
	for (auto pt : src->points) {
		if (tile == 0 || tile == pt.a) {
			dst->points.push_back(pt);
		}
	}
	return cwipc_from_pcl(dst, pc->timestamp(), NULL);
}

