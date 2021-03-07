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

    /// Unkown cell value
    const int UNKNOWN_CELL = 50;
    /// Occupied cell value
    const int OCCUPIED_CELL = 100;
    /// Uncoccupied cell value
    const int UNOCCUPIED_CELL = 0;

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
     * Returns a grid of the 9 cells including the index cell of a cell, if on the
     * border, outside cells will be marked -1 
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] index     Index of cells to get neighbours
     *                      
     * @param[out] neighboursGrid   neighbour grid, return values of neighbours in correct position, border will be -1
     */    
    std::vector<int8_t> getNeighboursGrid(nav_msgs::OccupancyGrid &ocgrid, int index);


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
     * Transforms a position from global domain to occupancy grid domain, checks if in grid
     *
     * @param[in] ocgrid     Input occupancy grid
     * @param[in] x     x position (meters)
     * @param[in] y     y position (meters)
     *                      
     * @param[out] index   Index of cell that contains position if in grid, else returns -1
     */
    int posToIndex(nav_msgs::OccupancyGrid &ocgrid, double x, double y);

    /**
     * Inflates occupied cells by a certain value
     *
     * @param[in] inflation     number of cells to inflate
     * @param[in] diagonalInflation     true = inflate diagonally, false = inflate horizontally and vertically only
     *                      
     * @param[in,out] ocgrid   Reference to ocgrid
     */
    void inflate(nav_msgs::OccupancyGrid &ocgrid, int inflation, int inflationCell, bool diagonalInflation = true);

    /**
     * Sets all cells that are not unoccupied to occupied
     *                     
     * @param[in,out] ocgrid   Reference to ocgrid
     */
    void unknownToOccupied(nav_msgs::OccupancyGrid &ocgrid);

    /**
     * Inflates from occupied cells to unoccupied cells by gradient
     *
     * @param[in] diagonalInflation     true = inflate diagonally, false = inflate horizontally and vertically only            
     * @param[in] ocgrid   Reference to ocgrid
     * 
     * @param[out] gradientInflatedGrid grid which has been inflated, occupied cells = 100, unoccupied cells = 0-100 based on proximity to occupied cells
     */
    std::vector<int8_t> gradientInflateGrid(nav_msgs::OccupancyGrid &ocgrid, bool diagonalInflation);

    /**
     * Computes the clusters of inputted value cells, returns grid with clusters labelled that same value, cells of other values will be labelled 0
     *
     * @param[in] clusterValue     value to calculate clusters for        
     * @param[in] ocgrid   Reference to ocgrid
     * 
     * @param[out] clusterGrid grid which has been clustered, cells of cluster value will be labelled by cluster, other cells will be 0
     */ 
    std::vector<int8_t> computeClusters(nav_msgs::OccupancyGrid &ocgrid, int clusterValue);

    /**
     * Skeletonises the unoccupied space forming a single cell thick path
     *
     * @param[in] diagonalInflation     true = inflate occupied cells in a diagonal direction, false = do not inflate occupied cells in diagonal direction
     * @param[in] diagonalSkeleton     true = diagonal connections are a path, false = connections can only be vertical or horizontal        
     * @param[in] ocgrid   Reference to ocgrid
     * 
     * @param[out] skeletonGrid grid which has been skeletonised, skeleton cells = 0, other = 100
     */ 
    std::vector<int8_t> skeletoniseUnoccupied(nav_msgs::OccupancyGrid &ocgrid, bool diagonalInflation, bool diagonalSkeleton);

    /**
     * Skeletonises the input space forming a single cell thick skeleton
     *
     * @param[in] skeletonValue     value to be skeletonised
     * @param[in] changeValue     value to change removed cells
     * @param[in] diagonalBoundary     true = diagonal boudnary expansion, false = only vertical or horizontal boundary expansion        
     * @param[in] ocgrid   Reference to ocgrid
     * 
     * @param[out] skeletonGrid grid which has been skeletonised, skeleton cells = 0, other = 100
     */ 
    void skeletonise(nav_msgs::OccupancyGrid &ocgrid, int skeletonValue, int changeValue, bool diagonalBoundary);

    void skeletonise2(nav_msgs::OccupancyGrid &ocgrid, int skeletonValue, int changeValue);
}