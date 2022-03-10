#ifndef LIDAR_PROCESSOR_PCL_POINT_TYPES_H
#define LIDAR_PROCESSOR_PCL_POINT_TYPES_H

#include <pcl/point_types.h>

namespace pcl 
{
    struct PointXYZIR
    {
        PCL_ADD_POINT4D;
        float intensity;
        std::uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZIR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
)
#endif // LIDAR_PROCESSOR_PCL_POINT_TYPES_H