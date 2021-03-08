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
nav_msgs::OccupancyGrid Ocgrid::generateOcgrid(int width, int height, double resolution, double xOrigin, double yOrigin, int initValue)
{
    nav_msgs::OccupancyGrid ocgrid;
    ocgrid.info.width = width;
    ocgrid.info.height = height;
    ocgrid.info.resolution = resolution;
    ocgrid.info.origin.position.x = xOrigin;
    ocgrid.info.origin.position.y = yOrigin;

    ocgrid.data = std::vector<int8_t>(width * height, initValue);
    return ocgrid;
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

    /// checks down up right left cells
    if (inGrid(ocgrid, row - 1, col))
        neighbours.push_back(indexFromRowCol(ocgrid, row - 1, col)); // lower cell
    if (inGrid(ocgrid, row + 1, col))
        neighbours.push_back(indexFromRowCol(ocgrid, row + 1, col)); // upper cell
    if (inGrid(ocgrid, row, col + 1))
        neighbours.push_back(indexFromRowCol(ocgrid, row, col + 1)); // right cell
    if (inGrid(ocgrid, row, col - 1))
        neighbours.push_back(indexFromRowCol(ocgrid, row, col - 1)); // left cell

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
 * Returns a grid of the 9 cells including the index cell of a cell, if on the
 * border, outside cells will be marked -1 
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] index     Index of cells to get neighbours
 *                      
 *  @param[out] neighboursGrid   neighbour grid, return values of neighbours in correct position, border will be -1
 */
std::vector<int8_t> Ocgrid::getNeighboursGrid(nav_msgs::OccupancyGrid &ocgrid, int index)
{
    std::vector<int8_t> neighbourGrid(9, 0);

    int row = rowFromIndex(ocgrid, index);
    int col = colFromIndex(ocgrid, index);

    int count = 0;
    for (int rowI = -1; rowI <= 1; rowI++)
    {
        for (int colI = -1; colI <= 1; colI++)
        {
            if (inGrid(ocgrid, row + rowI, col + colI))
                neighbourGrid[count] = ocgrid.data[Ocgrid::indexFromRowCol(ocgrid, row + rowI, col + colI)];
            else
                neighbourGrid[count] = -1;
            count++;
        }
    }
    return neighbourGrid;
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
 * Returns the indexs of the outer cells of the occupancy grid
 *
 * @param[in] ocgrid     Input occupancy grid
 *                      
 * @param[out] outerIndexs   Outer indexs of grid
 */
std::vector<int> Ocgrid::getOuterIndexs(nav_msgs::OccupancyGrid &ocgrid)
{
    std::vector<int> outerIndexs;
    int i = 0;
    //bottom
    outerIndexs.push_back(i);
    while (i < ocgrid.info.width - 1)
    {
        i++;
        outerIndexs.push_back(i);
    }
    //right
    while (i < ocgrid.info.width * ocgrid.info.height - 1)
    {
        i = i + ocgrid.info.width;
        outerIndexs.push_back(i);
    }
    //top
    while (i > ocgrid.info.width * (ocgrid.info.height - 1))
    {
        i--;
        outerIndexs.push_back(i);
    }
    //left
    while (i > ocgrid.info.width)
    {
        i = i - ocgrid.info.width;
        outerIndexs.push_back(i);
    }
    return outerIndexs;
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
 * Transforms a position from global domain to occupancy grid domain, checks if in grid
 *
 * @param[in] ocgrid     Input occupancy grid
 * @param[in] x     x position (meters)
 * @param[in] y     y position (meters)
 *                      
 * @param[out] index   Index of cell that contains position if in grid, else returns -1
 */
int Ocgrid::posToIndex(nav_msgs::OccupancyGrid &ocgrid, double x, double y)
{
    int col = std::floor((x - ocgrid.info.origin.position.x) / ocgrid.info.resolution);
    int row = std::floor((y - ocgrid.info.origin.position.y) / ocgrid.info.resolution);

    if (inGrid(ocgrid, row, col))
        indexFromRowCol(ocgrid, row, col);
    else
        return -1;
}

/**
 * Inflates occupied cells by a certain value
 *
 * @param[in] inflation     number of cells to inflate
 * @param[in] diagonalInflation     true = inflate diagonally, false = inflate horizontally and vertically only
 *                      
 * @param[in,out] ocgrid   Reference to ocgrid
 */
void Ocgrid::inflate(nav_msgs::OccupancyGrid &ocgrid, int inflation, int inflationCell, bool diagonalInflation)
{
    for (unsigned iteration = 0; iteration < inflation; iteration++)
    {
        std::vector<int> inflateIndexs;
        for (unsigned i = 0; i < ocgrid.data.size(); i++)
        {
            if (ocgrid.data[i] == inflationCell)
            {
                std::vector<int> neighbours = getNeighbours(ocgrid, i, diagonalInflation);
                inflateIndexs.insert(inflateIndexs.end(), neighbours.begin(), neighbours.end());
            }
        }
        for (auto i : inflateIndexs)
            ocgrid.data[i] = inflationCell;
    }
}

/**
 * Sets all cells that are not unoccupied to occupied
 *                   
 * @param[in,out] ocgrid   Reference to ocgrid
 */
void Ocgrid::unknownToOccupied(nav_msgs::OccupancyGrid &ocgrid)
{
    for (auto &&cell : ocgrid.data)
        if (static_cast<int>(cell) != UNOCCUPIED_CELL)
            cell = static_cast<int8_t>(OCCUPIED_CELL);
}

/**
 * Inflates from occupied cells to unoccupied cells by gradient
 * 
 * Expects a occupancy grid of only occupied and unoccupied cells
 *
 * @param[in] diagonalInflation     true = inflate diagonally, false = inflate horizontally and vertically only            
 * @param[in] ocgrid   Reference to ocgrid
 * 
 * @param[out] gradientskeletonGrid grid which has been inflated, occupied cells = 100, unoccupied cells = 0-100 based on proximity to occupied cells
 */
std::vector<int8_t> Ocgrid::gradientInflateGrid(nav_msgs::OccupancyGrid &ocgrid, bool diagonalInflation)
{
    std::vector<int8_t> gradientGrid = ocgrid.data;
    int level = 100;
    bool changed = true;
    while (changed)
    {
        level--;
        changed = false;
        // iterates through each square
        for (int unsigned i = 0; i < gradientGrid.size(); i++)
        {
            if (gradientGrid[i] == level + 1)
            {
                std::vector<int> neighbours = getNeighbours(ocgrid, i, diagonalInflation);
                for (auto n : neighbours)
                {
                    if (inGrid(ocgrid, n))
                    {
                        if (gradientGrid[n] == UNOCCUPIED_CELL)
                        {
                            gradientGrid[n] = level;
                            changed = true;
                        }
                    }
                }
            }
        }
    }
    level++;
    int delta = 100 - level;
    for (auto &&cell : gradientGrid)
        cell = (cell - level) * 100 / delta;
    return gradientGrid;
}

/**
 * Computes the clusters of inputted value cells, returns grid with clusters labelled that same value, cells of other values will be labelled 0
 *
 * @param[in] clusterValue     value to calculate clusters for        
 * @param[in] ocgrid   Reference to ocgrid
 * 
 * @param[out] clusterGrid grid which has been clustered, cells of cluster value will be labelled by cluster, other cells will be 0
 */
std::vector<int8_t> Ocgrid::computeClusters(nav_msgs::OccupancyGrid &ocgrid, int clusterValue)
{
    std::vector<int8_t> clusterGrid = ocgrid.data;
    int count = 0;
    for (unsigned i = 0; i < ocgrid.data.size(); i++)
    {
        if (clusterGrid[i] == clusterValue)
        {
            count++;
            clusterGrid[i] = count;
            std::queue<int> clusterQueue;
            clusterQueue.push(i);
            int currentI;
            while (!clusterQueue.empty())
            {
                currentI = clusterQueue.front();
                clusterQueue.pop();
                std::vector<int> neighbours = getNeighbours(ocgrid, currentI, true);
                for (auto n : neighbours)
                {
                    if (clusterGrid[n] == clusterValue)
                    {
                        clusterGrid[n] = count;
                        clusterQueue.push(n);
                    }
                }
            }
        }
    }
    for (unsigned i = 0; i < ocgrid.data.size(); i++)
        if (ocgrid.data[i] != clusterValue)
            clusterGrid[i] = 0;
    return clusterGrid;
}

/**
 * Skeletonises the unoccupied space forming a single cell thick path
 *
 * @param[in] diagonalInflation     true = inflate occupied cells in a diagonal direction, false = do not inflate occupied cells in diagonal direction
 * @param[in] diagonalSkeleton     true = diagonal connections are a path, false = connections can only be vertical or horizontal        
 * @param[in] ocgrid   Reference to ocgrid
 * 
 * @param[out] skeletonGrid grid which has been skeletonised, skeleton cells = 0, other = 100
 */
std::vector<int8_t> Ocgrid::skeletoniseUnoccupied(nav_msgs::OccupancyGrid &ocgrid, bool diagonalInflation, bool diagonalSkeleton)
{
    std::vector<int8_t> skeletonGrid = ocgrid.data;
    std::vector<int8_t> clusterGrid = computeClusters(ocgrid, OCCUPIED_CELL);
    std::queue<int> toExpand;
    for (unsigned i = 0; i < skeletonGrid.size(); i++)
        if (skeletonGrid[i] == OCCUPIED_CELL)
            toExpand.push(i);
    while (!toExpand.empty())
    {
        int index = toExpand.front();
        toExpand.pop();
        std::vector<int> expNeighbours = getNeighbours(ocgrid, index, diagonalInflation);
        for (auto expN : expNeighbours)
        {
            if (skeletonGrid[expN] == UNOCCUPIED_CELL)
            {
                bool cluster_hit = false;
                std::vector<int> pathNeighbours = getNeighbours(ocgrid, expN, !diagonalSkeleton);
                for (auto pathN : pathNeighbours)
                {
                    if ((skeletonGrid[pathN] == OCCUPIED_CELL) && clusterGrid[pathN] != clusterGrid[index])
                    {
                        cluster_hit = true;
                        break;
                    }
                }
                if (!cluster_hit)
                {
                    skeletonGrid[expN] = OCCUPIED_CELL;
                    clusterGrid[expN] = clusterGrid[index];
                    toExpand.push(expN);
                }
            }
        }
    }
    return skeletonGrid;
}

/**
 * Skeletonises the input space forming a single cell thick skeleton
 *
 * @param[in] skeletonValue     value to be skeletonised
 * @param[in] changeValue     value to change removed cells
 * @param[in] diagonalBoundary     true = diagonal boudnary expansion, false = only vertical or horizontal boundary expansion        
 * @param[in, out] ocgrid   Reference to ocgrid that will be skeletonised
 */
void Ocgrid::skeletonise(nav_msgs::OccupancyGrid &ocgrid, int skeletonValue, int changeValue)
{
    bool changed = true;
    nav_msgs::OccupancyGrid ocgridHold = ocgrid;
    
    /// gets vector of valid indexs for boundarys
    std::vector<int> validIndexs;
    for(unsigned i = 0; i < ocgrid.data.size(); i++) {
        if(ocgrid.data[i] == skeletonValue) validIndexs.push_back(i);
    }
    /// sets up boundaryIndexs vector
    std::vector<std::vector<int>> boundaryIndexs;
    boundaryIndexs.push_back(std::vector<int>());
    boundaryIndexs.push_back(std::vector<int>());

    while (changed)
    {
        changed = false;
        // iterates through valid Indexs vector, if boundary adds to boundary indexs and removes from valid
        boundaryIndexs[0].clear();
        boundaryIndexs[1].clear();
        auto it = validIndexs.begin();
        while (it != validIndexs.end())
        {
            bool boundaryAdded = false;
            int downI = Ocgrid::downIndex(ocgrid, *it);
            int upI = Ocgrid::upIndex(ocgrid, *it);
            int rightI = Ocgrid::rightIndex(ocgrid, *it);
            int leftI = Ocgrid::leftIndex(ocgrid, *it);
            if(Ocgrid::inGrid(ocgrid, downI) && ocgrid.data[downI] != skeletonValue) {
                boundaryIndexs[0].push_back(*it);
                boundaryAdded = true;
            }
            else if(Ocgrid::inGrid(ocgrid, upI) && ocgrid.data[upI] != skeletonValue) {
                boundaryIndexs[1].push_back(*it);
                boundaryAdded = true;
            }
            else if(Ocgrid::inGrid(ocgrid, rightI) && ocgrid.data[rightI] != skeletonValue) {
                boundaryIndexs[0].push_back(*it);
                boundaryAdded = true;
            }
            else if(Ocgrid::inGrid(ocgrid, leftI) && ocgrid.data[leftI] != skeletonValue) {
                boundaryIndexs[1].push_back(*it);
                boundaryAdded = true;
            }
            if(boundaryAdded) it = validIndexs.erase(it);
            else ++it;
        }
        
        // iterates through boundary cells
        for(auto b:boundaryIndexs) {
            ocgridHold = ocgrid;
            for(auto i:b) {
                // gets neighbour grid
                std::vector<int8_t> neighbourGridVec = getNeighboursGrid(ocgrid, i);
                std::vector<int8_t> neighbourGridVecHold = getNeighboursGrid(ocgridHold, i);
                nav_msgs::OccupancyGrid neighbourGrid;
                neighbourGrid.data = neighbourGridVec;
                neighbourGrid.info.height = 3;
                neighbourGrid.info.width = 3;
                // marks current boundary cell
                neighbourGrid.data[4] = changeValue;
                // counts amount of non marked cells
                int valCount = 0;
                for (auto &&val : neighbourGrid.data)
                {
                    if (val == -1)
                        val = skeletonValue;
                    if (val == skeletonValue)
                        valCount++;
                }
                int valCountHold = 0;
                for (auto &&val : neighbourGridVecHold)
                {
                    if (val == -1)
                        val = skeletonValue;
                    if (val == skeletonValue)
                        valCountHold++;
                }
                // count amount of clustered cells
                std::vector<int8_t> clusterGrid = Ocgrid::computeClusters(neighbourGrid, skeletonValue);
                int clusterCount = 0;
                for (auto val : clusterGrid)
                    if (val == 1) clusterCount++;
                // if equal changes cell
                if (valCount == clusterCount && valCountHold > 2)
                {
                    ocgrid.data[i] = changeValue;
                    changed = true;
                }
            }
        }    
    }
}

