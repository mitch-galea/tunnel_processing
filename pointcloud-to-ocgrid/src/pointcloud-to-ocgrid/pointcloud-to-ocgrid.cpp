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
     * @param[out] ocgridPtr Pointer for output pointcloud, also used for MapMetaData
     * @param[out] success indicator of whether process was successful
    */
bool PointcloudToOcgrid::convertPointcloudToOcgrid(PointCloud::Ptr pointcloudPtr, nav_msgs::OccupancyGrid::Ptr ocgridPtr) {
    /// iterates through data
    for(unsigned i = 0; i < ocgridPtr->data.size(); i++) {
        /// calculates the min and max x and y values using the index value, resolution and origin
        double x_min = ocgridPtr->info.origin.position.x + (i%ocgridPtr->info.width)*ocgridPtr->info.resolution;
        double x_max = x_min + ocgridPtr->info.resolution;

        double y_min = ocgridPtr->info.origin.position.y + std::floor(i/ocgridPtr->info.width)*ocgridPtr->info.resolution;
        double y_max = y_min + ocgridPtr->info.resolution;
    }
}


