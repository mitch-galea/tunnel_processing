/** @pointcloud-to-ocgrid.hpp
 *
 * Header file containing convertPointcloudToOcgridCoverter function
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>
#include <map>
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
     * for resolution, width, height and origin on Occupancy Grid, then options are to inflate and skeletonise the map to generate reliable clusters
     * then to remove small clusters, this is to manage noise and small obstacles such as stalactites
     * 
     * @param[in] pointcloudPtr  Pointer for input pointcloud
     * @param[in] inflation  Amount of inflation on occupied cells prior to skeletonisation, -1 input means inflation and skeletonisation wont occur
     * @param[in] minClusterSize  Minimum amount of cells for clusters, any clusters smaller will be removed, -1 value means cluster removal wont occur
     * 
     * @param[out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
     */
    void convertPointcloudToOcgrid(PointCloud::Ptr pointcloudPtr, nav_msgs::OccupancyGrid &ocgrid, int inflation, int minClusterSize);

    /**
     * Coverts point cloud to occupancy grid map
     * 
     * Checks if any pointcloud points are in the 2D region of the
     * occupancy grid and if so marks as occupied, uses ocgridPtr MapMetaData
     * for resolution, width, height and origin on Occupancy Grid
     * 
     * @param[in] pointcloudPtr  Pointer for input pointcloud
     * 
     * @param[out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
     */
    void pointsToOcgrid(PointCloud::Ptr pointcloudPtr, nav_msgs::OccupancyGrid &ocgrid);

    /**
     * Inflates the occupancy grid and then skeletonises to produce reliable clusters of cells
     * 
     * @param[in] inflation  Amount of inflation on occupied cells prior to skeletonisation
     * 
     * @param[in,out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
     */
    void inflateAndSkelotonise(nav_msgs::OccupancyGrid &ocgrid, int inflation);

    /**
     * Filters out clusters of occupied cells less than a certain size 
     * 
     * @param[in] minClusterSize  Minimum amount of cells for clusters, any clusters smaller will be removed, -1 value means cluster removal wont occur
     * 
     * @param[in,out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
     */
    void filterClusters(nav_msgs::OccupancyGrid &ocgrid, int minClusterSize);

    /**
     * Ray traces from 0 position to mark unnoccupied space
     * 
     * @param[in,out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
     */
    void raytrace(nav_msgs::OccupancyGrid &ocgrid);
}

