/** @ocgrid.cpp
 *
 * Implementation file with utility functions for occupancy grids
 */

#include "ocgrid/ocgrid.hpp"

/**
 * Generates occupancy grid map from files
 *
 * Uses YAML and image map file to generate an occupancy grid
 *
 * @param[in] mapFilePath     Full path to the YAML map file
 *                      
 * @param[out] ocgrid   The outputted occupancy grid map
 */
nav_msgs::OccupancyGrid Ocgrid::generateOcgrid(std::string mapFilePath) 
{
    std::string imageName;
    double resolution, occThresh, freeThresh;
    std::vector<double> originVec;
    bool negate;

    /// parses yaml file
    YAML::Node node = YAML::LoadFile(mapFilePath);
    if (!node["image"] ||
        !node["resolution"] ||
        !node["origin"] ||
        !node["occupied_thresh"] ||
        !node["free_thresh"] ||
        !node["negate"])
    {
        ROS_ERROR("OcGrid Constructor: YAML File Incorrect");
    }
    else
    {
        imageName = node["image"].as<std::string>();
        resolution = node["resolution"].as<double>();
        originVec = node["origin"].as<std::vector<double>>();
        occThresh = node["occupied_thresh"].as<double>();
        freeThresh = node["free_thresh"].as<double>();
        negate = static_cast<bool>(node["negate"].as<int>());

        /// gets double pointer for image function
        double *origin = &originVec[0];

        /// gets image path
        std::size_t found = mapFilePath.find_last_of("/\\");
        std::string imagePath = mapFilePath.substr(0, found + 1);
        imagePath.append(imageName);
        char *fname = new char[imagePath.length() + 1];
        std::strcpy(fname, imagePath.c_str());

        /// loads oc grid
        nav_msgs::GetMap::Response resp;
        map_server::loadMapFromFile(&resp, fname, resolution, negate, occThresh, freeThresh, origin);

        return resp.map;
    }
}

/**
 * Exports an image of the occupancy grid map
 *
 * @param[in] ocgrid     Input occupancy grid
 *                      
 * @param[in,out] image   Outputted image
 */
void Ocgrid::exportMapImage(nav_msgs::OccupancyGrid &ocgrid, cv::Mat &image)
{
    /// sets the image width and height based on the ocgrid
    image.create(ocgrid.info.height, ocgrid.info.width, CV_8UC3);
    for (unsigned row = 0; row < ocgrid.info.height; row++)
    {
        for (unsigned col = 0; col < ocgrid.info.width; col++)
        {
            cv::Vec3b value;
            int val = static_cast<int>(ocgrid.data[row * ocgrid.info.width + col]);
            /// assumes -1 values are unknown and sets to grey
            if (val == -1)
                val = 127;
            else
                val = (100 - val) * 255 / 100;
            /// sets image values to val so that image is greyscale
            value[0] = val;
            value[1] = val;
            value[2] = val;
            unsigned imRow = ocgrid.info.height - 1 - row;
            image.at<cv::Vec3b>(imRow, col) = value;
        }
    }
}

/**
 * Returns whether a cell index is located in the occupancy grid
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] index     Index of test cell
 *                      
 * @param[out] inGrid   true=cell in ocgrid, false= cell not in ocgrid
 */
bool Ocgrid::inGrid(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    return index >= 0 && index < ocgrid.data.size();
}

/**
 * Returns whether a cell index is located in the occupancy grid
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] row     row of inputted cell
 * @param[in] col     col of inputted cell
 *                      
 * @param[out] inGrid   true=cell in ocgrid, false= cell not in ocgrid
 */
bool Ocgrid::inGrid(nav_msgs::OccupancyGrid &ocgrid, int row, int col)
{
    return row >= 0 && row < ocgrid.info.height && col >= 0 && col < ocgrid.info.width;
}

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
bool Ocgrid::isNeighbours(nav_msgs::OccupancyGrid &ocgrid, int cell1, int cell2, 
                          bool checkDiagonals) 
{
    std::vector<int> neighbours = getNeighbours(ocgrid, cell1, checkDiagonals);
    /// iterates through cell1s neighbours and returns true if cell2 is found
    for (auto n : neighbours)
        if (n == cell2)
            return true;
    return false;
}

/**
 * Returns the indexs to all of a cells neighbours
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] index     Index of cells to get neighbours
 * @param[in] checkDiagonals    true=diagonals are checked, false=diagonals not checked
 *                      
 * @param[out] neighbours   Vector of indexs of neighbours of inputted cell index
 */
std::vector<int> Ocgrid::getNeighbours(nav_msgs::OccupancyGrid &ocgrid, int index, 
                            bool checkDiagonals)
{
    std::vector<int> neighbours;
    int row = rowFromIndex(ocgrid, index);
    int col = colFromIndex(ocgrid, index);

    /// checks left right up and down cells
    if (inGrid(ocgrid, row - 1, col))
        neighbours.push_back(indexFromRowCol(ocgrid, row - 1, col)); // lower cell
    if (inGrid(ocgrid, row, col - 1))
        neighbours.push_back(indexFromRowCol(ocgrid, row, col - 1)); // left cell
    if (inGrid(ocgrid, row, col + 1))
        neighbours.push_back(indexFromRowCol(ocgrid, row, col + 1)); // right cell
    if (inGrid(ocgrid, row + 1, col))
        neighbours.push_back(indexFromRowCol(ocgrid, row + 1, col)); // upper cell

    if (checkDiagonals)
    {
        if (inGrid(ocgrid, row - 1, col - 1))
            neighbours.push_back(indexFromRowCol(ocgrid, row - 1, col - 1)); // lower left cell
        if (inGrid(ocgrid, row - 1, col + 1))
            neighbours.push_back(indexFromRowCol(ocgrid, row - 1, col + 1)); // lower right cell
        if (inGrid(ocgrid, row + 1, col - 1))
            neighbours.push_back(indexFromRowCol(ocgrid, row + 1, col - 1)); // upper left cell
        if (inGrid(ocgrid, row + 1, col + 1))
            neighbours.push_back(indexFromRowCol(ocgrid, row + 1, col + 1)); // upper rigth cell
    }
    return neighbours;
}

/**
 * Transforms row and col domain to index domain
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] row     Row of inputted cell
 * @param[in] col     Col of inputted cell
 *                      
 * @param[out] index   Index of cell at inputted row and col
 */
int Ocgrid::indexFromRowCol(nav_msgs::OccupancyGrid &ocgrid, int row, int col)
{
    return row * ocgrid.info.width + col;
}

/**
 * Transforms index to row domain
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] index     Index of inputted cell
 *                      
 * @param[out] row   Row of cell at inputted index
 */
int Ocgrid::rowFromIndex(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    return floor(static_cast<double>(index) / static_cast<double>(ocgrid.info.width));
}

/**
 * Transforms index to col domain
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] index     Index of inputted cell
 *                      
 * @param[out] col   Col of cell at inputted index
 */
int Ocgrid::colFromIndex(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    return index % ocgrid.info.width;
}

/**
 * Returns the index to the cell above inputted cell
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] index     Index of inputted cell
 *                      
 * @param[out] upIndex   Index of upper cell or -1 if none
 */
int Ocgrid::upIndex(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    if (inGrid(ocgrid, rowFromIndex(ocgrid, index) + 1, colFromIndex(ocgrid, index)))
        return indexFromRowCol(ocgrid, rowFromIndex(ocgrid, index) + 1, colFromIndex(ocgrid, index));
    return -1;
}

/**
 * Returns the index to the cell below inputted cell
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] index     Index of inputted cell
 *                      
 * @param[out] downIndex   Index of down cell or -1 if none
 */
int Ocgrid::downIndex(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    if (inGrid(ocgrid, rowFromIndex(ocgrid, index) - 1, colFromIndex(ocgrid, index)))
        return indexFromRowCol(ocgrid, rowFromIndex(ocgrid, index) - 1, colFromIndex(ocgrid, index));
    return -1;
}

/**
 * Returns the index to the cell to the left of inputted cell
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] index     Index of inputted cell
 *                      
 * @param[out] leftIndex   Index of left cell or -1 if none
 */
int Ocgrid::leftIndex(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    if (inGrid(ocgrid, rowFromIndex(ocgrid, index), colFromIndex(ocgrid, index) - 1))
        return indexFromRowCol(ocgrid, rowFromIndex(ocgrid, index), colFromIndex(ocgrid, index) - 1);
    return -1;
}

/**
 * Returns the index to the cell to the right of inputted cell
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] index     Index of inputted cell
 *                      
 * @param[out] rightIndex   Index of right cell or -1 if none
 */
int Ocgrid::rightIndex(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    if (inGrid(ocgrid, rowFromIndex(ocgrid, index), colFromIndex(ocgrid, index) + 1))
        return indexFromRowCol(ocgrid, rowFromIndex(ocgrid, index), colFromIndex(ocgrid, index) + 1);
    return -1;
}

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
double Ocgrid::indexToX(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    return ocgrid.info.origin.position.x + (0.5 + static_cast<double>(colFromIndex(ocgrid, index))) * ocgrid.info.resolution;
}

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
double Ocgrid::indexToY(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    return ocgrid.info.origin.position.y + (0.5 + static_cast<double>(rowFromIndex(ocgrid, index))) * ocgrid.info.resolution;
}

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
int Ocgrid::posToIndex(nav_msgs::OccupancyGrid &ocgrid, double x, double y)
{
    int col = std::floor((x - ocgrid.info.origin.position.x) / ocgrid.info.resolution);
    int row = std::floor((y - ocgrid.info.origin.position.y) / ocgrid.info.resolution);
    return indexFromRowCol(ocgrid, row, col);
}