/** @ocgrid.h
 *
 * Header file with utility functions for occupancy grids
 */

#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <map_server/image_loader.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <queue>
#include <set>
#include <math.h>
#include <ros/ros.h>

namespace Ocgrid {
    /**
     * Generates occupancy grid map from files
     *
     * Uses YAML and image map file to generate an occupancy grid
     *
     * @param[in] mapFilePath     Full path to the YAML map file
     *                      
     * @param[out] ocgrid   The outputted occupancy grid map
     */
    nav_msgs::OccupancyGrid generateOcgrid(std::string mapFilePath);

    /**
     * Generates occupancy grid map from files
     *
     * Uses YAML and image map file to generate an occupancy grid
     *
     * @param[in] width     Width (cells) of the occupancy grid
     * @param[in] height     Height (cells) of the occupancy grid
     * @param[in] resolution     Resolution (m/cell) of the occupancy grid
     * @param[in] xOrigin     X position of bottom left cell of the occupancy grid
     * @param[in] yOrigin    Y position of bottom left cell of the occupancy grid
     * @param[in] initValue     Initial value of all cells, default value in 0
     *                      
     * @param[out] ocgrid   The outputted occupancy grid map
     */
    nav_msgs::OccupancyGrid generateOcgrid(int width, int height, double resolution, double xOrigin, double yOrigin, int initValue = 0);
    
    /**
     * Exports an image of the occupancy grid map
     *
     * @param[in] ocgrid     Input occupancy grid
     *                      
     * @param[in,out] image   Outputted image
     */
    void exportMapImage(nav_msgs::OccupancyGrid &ocgrid, cv::Mat &image);
    
    /**
     * Returns whether a cell index is located in the occupancy grid
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of test cell
     *                      
     * @param[out] inGrid   true=cell in ocgrid, false= cell not in ocgrid
     */
    bool inGrid(nav_msgs::OccupancyGrid &ocgrid, int index);

    /**
     * Returns whether a cell index is located in the occupancy grid
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] row     row of inputted cell
     * @param[in] col     col of inputted cell
     *                      
     * @param[out] inGrid   true=cell in ocgrid, false= cell not in ocgrid
     */    
    bool inGrid(nav_msgs::OccupancyGrid &ocgrid, int row, int col);

    /**
     * Returns whether 2 cells are neighbours
     * 
     * Takes 2 input cell indexs and checks whether the cells are neighbours
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] cell1     Index of cell for neighbour test
     * @param[in] cell2     Index of second cell for neighbour test
     * @param[in] checkDiagonals    true=diagonals are checked, false=diagonals not checked  
     *                      
     * @param[out] isNeighbours   true=cells are neighbousr, false= cells are not neighbours
     */
    bool isNeighbours(nav_msgs::OccupancyGrid &ocgrid, int cell1, int cell2, bool checkDiagonals=true);

    /**
     * Returns the indexs to all of a cells neighbours
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of cells to get neighbours
     * @param[in] checkDiagonals    true=diagonals are checked, false=diagonals not checked
     *                      
     * @param[out] neighbours   Vector of indexs of neighbours of inputted cell index
     */    
    std::vector<int> getNeighbours(nav_msgs::OccupancyGrid &ocgrid, int index, bool checkDiagonals=false);

    /**
     * Transforms row and col domain to index domain
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] row     Row of inputted cell
     * @param[in] col     Col of inputted cell
     *                      
     * @param[out] index   Index of cell at inputted row and col
     */
    int indexFromRowCol(nav_msgs::OccupancyGrid &ocgrid, int row, int col);
    
    /**
     * Transforms index to row domain
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of inputted cell
     *                      
     * @param[out] row   Row of cell at inputted index
     */
    int rowFromIndex(nav_msgs::OccupancyGrid &ocgrid, int index);
    
    /**
     * Transforms index to col domain
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of inputted cell
     *                      
     * @param[out] col   Col of cell at inputted index
     */
    int colFromIndex(nav_msgs::OccupancyGrid &ocgrid, int index);
    
    /**
     * Returns the index to the cell above inputted cell
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of inputted cell
     *                      
     * @param[out] upIndex   Index of upper cell or -1 if none
     */
    int upIndex(nav_msgs::OccupancyGrid &ocgrid, int index);
    
    /**
     * Returns the index to the cell below inputted cell
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of inputted cell
     *                      
     * @param[out] downIndex   Index of down cell or -1 if none
     */
    int downIndex(nav_msgs::OccupancyGrid &ocgrid, int index);
    
    /**
     * Returns the index to the cell to the left of inputted cell
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of inputted cell
     *                      
     * @param[out] leftIndex   Index of left cell or -1 if none
     */
    int leftIndex(nav_msgs::OccupancyGrid &ocgrid, int index);

    /**
     * Returns the index to the cell to the right of inputted cell
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of inputted cell
     *                      
     * @param[out] rightIndex   Index of right cell or -1 if none
     */
    int rightIndex(nav_msgs::OccupancyGrid &ocgrid, int index);

    /**
     * Returns the indexs of the outer cells of the occupancy grid
     *
     * @param[in] ocgrid     Input occupancy grid
     *                      
     * @param[out] outerIndexs   Outer indexs of grid
     */
    std::vector<int> getOuterIndexs(nav_msgs::OccupancyGrid &ocgrid);

    /**
     * Transforms from occupancy grid domain to global x position
     *
     * Returns the X position (meters) of the centre position of the cell,
     * using the origin of the map and resolution
     * 
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of inputted cell
     *                      
     * @param[out] x   x position (meters) of the centre position of the cell
     */
    double indexToX(nav_msgs::OccupancyGrid &ocgrid, int index);
    
    /**
     * Transforms from occupancy grid domain to global y position
     *
     * Returns the Y position (meters) of the centre position of the cell,
     * using the origin of the map and resolution
     * 
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of inputted cell
     *                      
     * @param[out] y   y position (meters) of the centre position of the cell
     */
    double indexToY(nav_msgs::OccupancyGrid &ocgrid, int index);
    
    /**
     * Transforms from global domain to occupancy grid domain
     * 
     * Transforms a position from global domain to occupancy grid domain, 
     * does not check whether index is within grid
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] x     x position (meters)
     * @param[in] y     y position (meters)
     *                      
     * @param[out] index   Index of cell that contains position
     */
    int posToIndex(nav_msgs::OccupancyGrid &ocgrid, double x, double y);

}