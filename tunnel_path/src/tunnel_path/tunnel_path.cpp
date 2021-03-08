

/** @pointcloud-to-ocgrid-converter.cpp
 *
 * Implementation file containing convertPointcloudToOcgrid function
 */

#include "tunnel_path/tunnel_path.hpp"

nav_msgs::GridCells TunnelPath::computeTunnelPath(nav_msgs::OccupancyGrid &ocgrid) {
    /// converts unknown squares to occupied
    Ocgrid::unknownToOccupied(ocgrid);

    /// skeletonises Unoccupied space
    Ocgrid::skeletonise(ocgrid, Ocgrid::UNOCCUPIED_CELL, Ocgrid::OCCUPIED_CELL);

    /// calculates grid cells
    nav_msgs::GridCells gridCells;
    gridCells.cell_height = ocgrid.info.resolution;
    gridCells.cell_width = ocgrid.info.resolution;
    for(unsigned i = 0; i < ocgrid.data.size(); i++) {
        if(ocgrid.data[i] == Ocgrid::UNOCCUPIED_CELL) {
            geometry_msgs::Point p;
            p.x = Ocgrid::indexToX(ocgrid, i);
            p.y = Ocgrid::indexToY(ocgrid, i);
            gridCells.cells.push_back(p);
        }
    }
    return gridCells;
}




