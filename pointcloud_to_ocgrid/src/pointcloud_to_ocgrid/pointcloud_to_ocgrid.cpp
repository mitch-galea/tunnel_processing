/** @pointcloud-to-ocgrid-converter.cpp
 *
 * Implementation file containing convertPointcloudToOcgrid function
 */

#include "pointcloud_to_ocgrid/pointcloud_to_ocgrid.hpp"

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
void PointcloudToOcgrid::convertPointcloudToOcgrid(PointCloud::Ptr pointcloudPtr, nav_msgs::OccupancyGrid &ocgrid,
                                                    double xOrigin, double yOrigin) 
{
    std::cout << "H: " << ocgrid.info.height << ", W: " << ocgrid.info.width << ", DS: " << ocgrid.data.size() << std::endl;

    /// sets all cells to unknown
    std::fill(ocgrid.data.begin(), ocgrid.data.end(), UNKNOWN_CELL);

    ROS_INFO("FILLED");

    int index;
    /// iterates through data and sets any grid to occupied with points in them
    for(auto p:pointcloudPtr->points) {
        index = Ocgrid::posToIndex(ocgrid, p.x, p.y);
        if(Ocgrid::inGrid(ocgrid, index)) ocgrid.data[index] = 100;
    }

    ROS_INFO("OCCUPIED MARKED");

    // Ray Tracing
    int originIndex = Ocgrid::posToIndex(ocgrid, xOrigin, yOrigin);
    xOrigin = Ocgrid::indexToX(ocgrid, originIndex);
    yOrigin = Ocgrid::indexToY(ocgrid, originIndex);

    ocgrid.data[originIndex] = UNOCCUPIED_CELL;

    std::vector<int> outerIndexs = Ocgrid::getOuterIndexs(ocgrid);

    double xOuter, yOuter;
    std::vector<double> xLines, yLines;
    for(unsigned i = 1; i < ocgrid.info.width; i++) {
        xLines.push_back(ocgrid.info.origin.position.x + i*ocgrid.info.resolution);
    }
    for(unsigned i = 1; i < ocgrid.info.height; i++) {
        yLines.push_back(ocgrid.info.origin.position.y + i*ocgrid.info.resolution);
    }

    for(auto outerIndex:outerIndexs) {
        xOuter = Ocgrid::indexToX(ocgrid, outerIndex);
        yOuter = Ocgrid::indexToY(ocgrid, outerIndex);
        double angle = atan2(yOuter-yOrigin, xOuter-xOrigin);

        double xMin = std::min(xOrigin, xOuter);
        double xMax = std::max(xOrigin, xOuter);
        double yMin = std::min(yOrigin, yOuter);
        double yMax = std::max(yOrigin, yOuter);
        
        std::vector<std::pair<double, bool>> intersections;

        for(auto xLine:xLines) {
            if(xLine > xMin && xLine < xMax) {
                double l = (xLine-xOrigin) / cos(angle);
                intersections.push_back(std::pair<double,bool>(l, true));
            }
        }
        for(auto yLine:yLines) {
            if(yLine > yMin && yLine < yMax) {
                double l = (yLine-yOrigin) / sin(angle);
                intersections.push_back(std::pair<double,bool>(l, false));
            }
        }
        std::sort(intersections.begin(), intersections.end());

        int currentIndex = originIndex;

        for(auto intersection:intersections) {
            if(intersection.second == true) {
                if(fabs(angle) < M_PI_2){
                    currentIndex = Ocgrid::rightIndex(ocgrid, currentIndex);
                } else {
                    currentIndex = Ocgrid::leftIndex(ocgrid, currentIndex);
                }
            } else {
                if(angle > 0.0){
                    currentIndex = Ocgrid::upIndex(ocgrid, currentIndex);
                } else {
                    currentIndex = Ocgrid::downIndex(ocgrid, currentIndex);
                }
            }
            if(ocgrid.data[currentIndex] == OCCUPIED_CELL) break;
            else ocgrid.data[currentIndex] = UNOCCUPIED_CELL;
        }
    }

    ROS_INFO("RAYS TRACED");
}


