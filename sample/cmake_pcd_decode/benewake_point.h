#ifndef BENEWAKE_POINT_H
#define BENEWAKE_POINT_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace benewake
{

struct EIGEN_ALIGN16 PointXYZI_EXT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    double timestamp;
    uint8_t channel;
    uint8_t confidence;
    uint16_t row;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

POINT_CLOUD_REGISTER_POINT_STRUCT(benewake::PointXYZI_EXT,
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, intensity, intensity)
                                (double, timestamp, timestamp)
                                (uint8_t, channel, channel)
                                (uint8_t, confidence, confidence)
                                (uint16_t, row, row)
)

typedef benewake::PointXYZI_EXT bwPCLPoint;
typedef pcl::PointCloud<bwPCLPoint> bwPCLPointCloud;

#endif // BENEWAKE_POINT_H
