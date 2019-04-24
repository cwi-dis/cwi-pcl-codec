#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#define PCL_INSTALLED // xxxjack temp
#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>
#include <pcl/cloud_codec_v2/impl/point_cloud_codec_v2_impl.hpp>

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"

namespace pcl {

namespace io {
template class OctreePointCloudCodecV2<cwipc_pcl_point>;
}

template class RadiusOutlierRemoval<cwipc_pcl_point>;
}
