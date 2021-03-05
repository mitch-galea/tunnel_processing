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
 * for resolution, width, height and origin on Occupancy Grid, then options are to inflate and skeletonise the map to generate reliable clusters
 * then to remove small clusters, this is to manage noise and small obstacles such as stalactites
 * 
 * @param[in] pointcloudPtr  Pointer for input pointcloud
 * @param[in] inflation  Amount of inflation on occupied cells prior to skeletonisation, 0 input means inflation and skeletonisation wont occur
 * @param[in] minClusterSize  Minimum amount of cells for clusters, any clusters smaller will be removed, 0 value means cluster removal wont occur
 * 
 * @param[out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
 */
void PointcloudToOcgrid::convertPointcloudToOcgrid(PointCloud::Ptr pointcloudPtr, nav_msgs::OccupancyGrid &ocgrid, int inflation, int minClusterSize) 
{
    // /// Marks point cloud points as occupied in ocgrid
    // pointsToOcgrid(pointcloudPtr, ocgrid);
    // /// if inflation value is > 0 will inflate and skeletonise
    // if(inflation > 0) inflateAndSkelotonise(ocgrid, inflation);
    // /// if minClusterSize value is > 0 it will filter clusters
    // if(minClusterSize > 0) filterClusters(ocgrid, minClusterSize);
    // /// ray traces
    // raytrace(ocgrid);

    /// sets all cells to unknown
    std::fill(ocgrid.data.begin(), ocgrid.data.end(), Ocgrid::UNKNOWN_CELL);

    int index;
    /// iterates through data and sets any grid to occupied with points in them
    for(auto p:pointcloudPtr->points) {
        index = Ocgrid::posToIndex(ocgrid, p.x, p.y);
        if(Ocgrid::inGrid(ocgrid, index)) ocgrid.data[index] = Ocgrid::OCCUPIED_CELL;
    }

    
    if(inflation > 0) {
        Ocgrid::inflate(ocgrid, inflation, Ocgrid::OCCUPIED_CELL, true);
        //Ocgrid::skeletonise(ocgrid, Ocgrid::OCCUPIED_CELL, Ocgrid::UNKNOWN_CELL, false);
    }


    // if(minClusterSize > 0) {
    //     /// generates cluster grid
    //     std::vector<int8_t> clusterGrid = Ocgrid::computeClusters(ocgrid, Ocgrid::OCCUPIED_CELL);
    //     /// counts each cluster in the map
    //     std::map<int, int> clusterCount;
    //     for(auto clusterVal:clusterGrid) {
    //         if(clusterCount.find(clusterVal) == clusterCount.end()) clusterCount[clusterVal] = 0; 
    //         else clusterCount[clusterVal] ++;
    //     }
    //     /// iterates through each ocgrid cell and checks cluster size, if below minClusterSize removes from ocgrid
    //     for(unsigned i = 0; i < ocgrid.data.size(); i ++) {
    //         if(clusterCount[clusterGrid[i]] < minClusterSize) ocgrid.data[i] = Ocgrid::UNKNOWN_CELL;
    //     }
    // }

    // Ray Tracing
    double xOrigin = 0.0, yOrigin = 0.0;
    int originIndex = Ocgrid::posToIndex(ocgrid, xOrigin, yOrigin);
    xOrigin = Ocgrid::indexToX(ocgrid, originIndex);
    yOrigin = Ocgrid::indexToY(ocgrid, originIndex);

    ocgrid.data[originIndex] = Ocgrid::UNOCCUPIED_CELL;

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
            if(ocgrid.data[currentIndex] == Ocgrid::OCCUPIED_CELL) break;
            else ocgrid.data[currentIndex] = Ocgrid::UNOCCUPIED_CELL;
        }
    }




}

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
void pointsToOcgrid(PointCloud::Ptr pointcloudPtr, nav_msgs::OccupancyGrid &ocgrid) {
    /// sets all cells to unknown
    std::fill(ocgrid.data.begin(), ocgrid.data.end(), Ocgrid::UNKNOWN_CELL);

    int index;
    /// iterates through data and sets any grid to occupied with points in them
    for(auto p:pointcloudPtr->points) {
        index = Ocgrid::posToIndex(ocgrid, p.x, p.y);
        if(Ocgrid::inGrid(ocgrid, index)) ocgrid.data[index] = Ocgrid::OCCUPIED_CELL;
    }
}

/**
 * Inflates the occupancy grid and then skeletonises to produce reliable clusters of cells
 * 
 * @param[in] inflation  Amount of inflation on occupied cells prior to skeletonisation
 * 
 * @param[in,out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
 */
void inflateAndSkelotonise(nav_msgs::OccupancyGrid &ocgrid, int inflation) {
    Ocgrid::inflate(ocgrid, inflation, Ocgrid::OCCUPIED_CELL, true);
    Ocgrid::skeletonise(ocgrid, Ocgrid::OCCUPIED_CELL, Ocgrid::UNKNOWN_CELL, false);
}

/**
 * Filters out clusters of occupied cells less than a certain size 
 * 
 * @param[in] minClusterSize  Minimum amount of cells for clusters, any clusters smaller will be removed, -1 value means cluster removal wont occur
 * 
 * @param[in,out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
 */
void filterClusters(nav_msgs::OccupancyGrid &ocgrid, int minClusterSize) {
    /// generates cluster grid
    std::vector<int8_t> clusterGrid = Ocgrid::computeClusters(ocgrid, Ocgrid::OCCUPIED_CELL);
    /// counts each cluster in the map
    std::map<int, int> clusterCount;
    for(auto clusterVal:clusterGrid) {
        if(clusterCount.find(clusterVal) == clusterCount.end()) clusterCount[clusterVal] = 0; 
        else clusterCount[clusterVal] ++;
    }
    /// iterates through each ocgrid cell and checks cluster size, if below minClusterSize removes from ocgrid
    for(unsigned i = 0; i < ocgrid.data.size(); i ++) {
        if(clusterCount[clusterGrid[i]] < minClusterSize) ocgrid.data[i] = Ocgrid::UNKNOWN_CELL;
    }

}

/**
 * Ray traces from 0 position to mark unnoccupied space
 * 
 * @param[in,out] ocgrid   Occupany Grid for output pointcloud, also used for MapMetaData
 */
void raytrace(nav_msgs::OccupancyGrid &ocgrid) {
    // Ray Tracing
    double xOrigin = 0.0, yOrigin = 0.0;
    int originIndex = Ocgrid::posToIndex(ocgrid, xOrigin, yOrigin);
    xOrigin = Ocgrid::indexToX(ocgrid, originIndex);
    yOrigin = Ocgrid::indexToY(ocgrid, originIndex);

    ocgrid.data[originIndex] = Ocgrid::UNOCCUPIED_CELL;

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
            if(ocgrid.data[currentIndex] == Ocgrid::OCCUPIED_CELL) break;
            else ocgrid.data[currentIndex] = Ocgrid::UNOCCUPIED_CELL;
        }
    }
}


