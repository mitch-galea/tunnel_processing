/** @pointcloud-to-ocgrid-converter.cpp
 *
 * Implementation file containing convertPointcloudToOcgrid function
 */

#include "pointcloud-to-ocgrid/pointcloud-to-ocgrid.hpp"

/**
     * Coverts point cloud to occupancy grid map
     * 
     * Checks if any pointcloud points are in the 2D region of the
     * occupancy grid and if so marks as occupied, uses ocgridPtr MapMetaData
     * for resolution, width, height and origin on Occupancy Grid
     * 
     * @param[in] pointcloudPtr  Pointer for input pointcloud
     * @param[out] ocgrid occupancy grid for output pointcloud, also used for MapMetaData
     * @param[out] success indicator of whether process was successful
    */
bool PointcloudToOcgrid::convertPointcloudToOcgrid(PointCloud::Ptr pointcloudPtr, nav_msgs::OccupancyGrid ocgrid) 
{
    int index;
    /// iterates through data
    for(auto p:pointcloudPtr->points) {
        //index = ocgrid.posToIndex(p.x, p.y);
        //if(ocgrid.inGrid(index)) ocgrid.ocgrid->data[index] = 100;
    }
}


