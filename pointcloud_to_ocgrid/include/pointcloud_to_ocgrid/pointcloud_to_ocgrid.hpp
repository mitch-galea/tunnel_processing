/** @pointcloud-to-ocgrid.hpp
 *
 * Header file containing convertPointcloudToOcgridCoverter function
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>
#include <ocgrid/ocgrid.hpp>

/**
 * In this implementation only PXYZI point types (I being intensity) are supported, 
 * this is the pointcloud output from typical lidar sensors
 */
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

/**
 * Represents a Pointcloud to Ocgrid Converter
 *
 * Pointcloud to Ocgrid converter takes will convert inputted
 * point cloud data to occupancy grid maps
 *
 */

namespace PointcloudToOcgrid {
    /**
     * Coverts point cloud to occupancy grid map
     * 
     * Checks if any pointcloud points are in the 2D region of the
     * occupancy grid and if so marks as occupied, uses ocgridPtr MapMetaData
     * for resolution, width, height and origin on Occupancy Grid
     * 
     * @param[in] pointcloudPtr  Pointer for input pointcloud
     * @param[in] inflation  Amount of inflation on occupied cells
     * 
     * @param[out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
     */
    void convertPointcloudToOcgrid(PointCloud::Ptr pointcloudPtr, nav_msgs::OccupancyGrid &ocgrid, int inflation);
}

