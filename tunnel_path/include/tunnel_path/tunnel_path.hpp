/** @pointcloud-to-ocgrid.hpp
 *
 * Header file containing convertPointcloudToOcgridCoverter function
 */

#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>
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
     * @param[in] 
     * 
     * @param[in, out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
     */
    nav_msgs::GridCells computeTunnelPath(nav_msgs::OccupancyGrid &ocgrid);
}

