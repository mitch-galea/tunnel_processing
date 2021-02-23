//TODO DOCUMENTATION

#ifndef OCGRID_H
#define OCGRID_H

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <map_server/image_loader.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <queue>
#include <set>
#include <math.h>

class OcGrid {
private:

public:
    static nav_msgs::OccupancyGrid generateOcgrid(std::string map_file_path);
    // occupancy grid msg - data
    nav_msgs::OccupancyGrid::Ptr ocgrid;

    // constructor from ros msg
    OcGrid(nav_msgs::OccupancyGrid::Ptr oc_grid_ptr);
    // // constructor that uses map server image loader function and yaml file
    // explicit OcGrid(std::string map_file_path);

    // exports map image
    void exportMapImage(cv::Mat &image);
    // returns outer indexs of map
    std::vector<int> getOuterIndexs();

    // returns whether cell is in grid
    bool inGrid(int index);
    bool inGrid(int row, int col);
    // returns whether 2 cells are neighbours
    bool neighbours(int cell_1, int cell_2, bool diagonal=true);
    // returns a vector of ints of the squares neighbours
    std::vector<int> getNeighbours(int index, bool diagonal=false);
    //returns the index of a cell from row and column
    int indexFromRowCol(int row, int col);
    // returns row from index
    int rowFromIndex(int index);
    // returns col from index
    int colFromIndex(int index);
    // returns index to up cell or -1 if none
    int upIndex(int index);
    // returns index to down cell or -1 if none
    int downIndex(int index);
    // returns index to left cell or -1 if none
    int leftIndex(int index);
    // returns index to right cell or -1 if none
    int rightIndex(int index);

    // transforms grid index to map pose
    double indexToX(int index);
    // transforms grid index to map pose using col and row
    double indexToY(int index);
    // transforms pos to grid index
    int posToIndex(double x, double y);


};


#endif //OCGRID_H
