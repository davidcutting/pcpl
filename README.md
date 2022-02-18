# lidar_processor
A simple PointCloud processor for ground based robots using a 3D LiDAR.

## Major Components

* **Passthrough Filter** - Cropping PointClouds to a range of acceptable and usable values.
* **Voxel Filter** - Downsampling PointClouds to speed up more computationally intensive filters.
* **Segementation** - Separating parts of PointClouds to ease in map building. Examples include: Ground Points, Obstacle Points, Overhead Points
* **Post-Processor** - Using segemented PointCloud indicies to generate new fully filtered PointClouds, or doing a 2D projection for scan based mapping.
* **PointCloud-Concatenate** - Merge multiple sources of Pointclouds into one Pointcloud.
![PointCloud Pipeline](doc/lidar_processor_architecture.png "PointCloud Pipeline")

## Additional Info

This can be used to fix strange ign_gazebo frame_ids which come from a URDF robot model being spawned in to the simulator with an attached gpu_ray.
