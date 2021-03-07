/** @pointcloud-to-ocgrid.hpp
 *
 * Header file containing convertPointcloudToOcgridCoverter function
 */

#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>
#include <map>
#include <ocgrid/ocgrid.hpp>

/**
 * Represents a Pointcloud to Ocgrid Converter
 *
 * Pointcloud to Ocgrid converter takes will convert inputted
 * point cloud data to occupancy grid maps
 *
 */

namespace TunnelPath {
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
}

