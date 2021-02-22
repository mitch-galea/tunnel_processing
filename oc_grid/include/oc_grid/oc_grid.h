#ifndef OCGRID_H
#define OCGRID_H

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <map_server/image_loader.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <queue>
#include <set>
#include <math.h>

class OcGrid {
private:

public:
    // occupancy grid msg - data
    nav_msgs::OccupancyGrid oc_grid;
    // grid to store gradient grid
    std::vector<int8_t> gradient_grid;
    // grid to store cluster grid
    std::vector<int8_t> cluster_grid;
    // grid to store inflated grid
    std::vector<int8_t> inflated_grid;

    // constructor from ros msg
    explicit OcGrid(nav_msgs::OccupancyGrid oc_grid_in);
    // constructor that uses map server image loader function and yaml file
    explicit OcGrid(std::string map_file_path);

    // turns unknown cells to occupied
    void unknownToOccupied();
    // exports map image
    void exportMapImage(cv::Mat &image, int mode=0);
    // exports map image
    void exportMapImage(cv::Mat &image, std::vector<int> grid_cells, std::vector<cv::Vec3b> colours, int mode=0);
    // gradual inflates map
    void gradientInflate(bool diagonal);
    // cluster occupied
    void clusterOccupied();
    // inflates map till single width path
    void inflate(bool d_inflate, bool d_path);
    // returns vector of outer cell indexs order starting with bottom-left cell and counterclockwise
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

    // transforms grid position to map pose
    geometry_msgs::Pose gridToPose(int index);
    // transforms grid position to map pose using col and row
    geometry_msgs::Pose gridToPose(int col, int row);
    // transforms pose to grid position
    int poseToGrid(geometry_msgs::Pose);


};


#endif //OCGRID_H
