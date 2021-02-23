//TODO DOCUMENTATION

#include "oc-grid/oc-grid.h"

nav_msgs::OccupancyGrid OcGrid::generateOcgrid(std::string map_file_path) {
    std::string image_name;
    double resolution, occ_thresh, free_thresh;
    std::vector<double> origin_vec;
    bool negate;
    YAML::Node node = YAML::LoadFile(map_file_path);
    if (!node["image"] ||
        !node["resolution"] ||
        !node["origin"] ||
        !node["occupied_thresh"] ||
        !node["free_thresh"] ||
        !node["negate"]) {
        ROS_ERROR("OcGrid Constructor: YAML File Incorrect");
    } else {
        image_name = node["image"].as<std::string>();
        resolution = node["resolution"].as<double>();
        origin_vec = node["origin"].as<std::vector<double>>();
        occ_thresh = node["occupied_thresh"].as<double>();
        free_thresh = node["free_thresh"].as<double>();
        negate = static_cast<bool>(node["negate"].as<int>());

        // gets double pointer for image function
        double* origin = &origin_vec[0];

        // gets image path
        std::size_t found = map_file_path.find_last_of("/\\");
        std::string image_path = map_file_path.substr(0,found+1);
        image_path.append(image_name);
        char * fname = new char [image_path.length()+1];
        std::strcpy (fname, image_path.c_str());

        // loads oc grid
        nav_msgs::GetMap::Response resp;
        map_server::loadMapFromFile(&resp, fname, resolution, negate, occ_thresh, free_thresh, origin);

        return resp.map;
    }
}

OcGrid::OcGrid(nav_msgs::OccupancyGrid::Ptr oc_grid_ptr): ocgrid(oc_grid_ptr)
{}

void OcGrid::exportMapImage(cv::Mat &image) {
    image.create(ocgrid->info.height, ocgrid->info.width, CV_8UC3);
    for(unsigned row = 0; row < ocgrid->info.height; row++) {
        for(unsigned col = 0; col < ocgrid->info.width; col++) {
            cv::Vec3b value;
            int val = static_cast<int>(ocgrid->data[row*ocgrid->info.width + col]);
            if(val == -1) val = 127;
            else val = (100-val)*255/100;

            value[0] = val;
            value[1] = val;
            value[2] = val;

            unsigned im_row = ocgrid->info.height - 1 - row;
            image.at<cv::Vec3b>(im_row, col) = value;
        }
    }
}

std::vector<int> OcGrid::getOuterIndexs() {
    std::vector<int> outer_indexs;
    int i = 0;
    //bottom
    outer_indexs.push_back(i);
    while(i < ocgrid->info.width - 1) {
        i++;
        outer_indexs.push_back(i);
    }
    //right
    while(i < ocgrid->info.width*ocgrid->info.height-1) {
        i = i+ocgrid->info.width;
        outer_indexs.push_back(i);
    }
    //top
    while(i > ocgrid->info.width*(ocgrid->info.height-1)) {
        i--;
        outer_indexs.push_back(i);
    }
    //left
    while(i > ocgrid->info.width) {
        i = i-ocgrid->info.width;
        outer_indexs.push_back(i);
    }
    return outer_indexs;
}


bool OcGrid::inGrid(int index) {
    return index >= 0 && index < ocgrid->data.size();
}

bool OcGrid::inGrid(int row, int col) {
    return row >= 0 && row < ocgrid->info.height && col >= 0 && col < ocgrid->info.width;
}

int OcGrid::indexFromRowCol(int row, int col) {
    return row*ocgrid->info.width + col;
}

int OcGrid::rowFromIndex(int index) {
    return floor(static_cast<double>(index)/ static_cast<double>(ocgrid->info.width));
}

int OcGrid::colFromIndex(int index) {
    return index % ocgrid->info.width;
}

std::vector<int> OcGrid::getNeighbours(int index, bool diagonal) {
    if(index >= ocgrid->data.size()) ROS_ERROR("OcGrid getNeighbours: Index exceeds data");
    std::vector<int> neighbours;
    int row = rowFromIndex(index);
    int col = colFromIndex(index);

    if(inGrid(row-1, col))neighbours.push_back(indexFromRowCol(row-1, col)); // lower cell
    if(inGrid(row, col-1))neighbours.push_back(indexFromRowCol(row, col-1)); // left cell
    if(inGrid(row, col+1))neighbours.push_back(indexFromRowCol(row, col+1)); // right cell
    if(inGrid(row+1, col))neighbours.push_back(indexFromRowCol(row+1, col)); // upper cell

    if(diagonal) {
        if(inGrid(row-1, col-1))neighbours.push_back(indexFromRowCol(row-1, col-1)); // lower left cell
        if(inGrid(row-1, col+1))neighbours.push_back(indexFromRowCol(row-1, col+1)); // lower right cell
        if(inGrid(row+1, col-1))neighbours.push_back(indexFromRowCol(row+1, col-1)); // upper left cell
        if(inGrid(row+1, col+1))neighbours.push_back(indexFromRowCol(row+1, col+1)); // upper rigth cell
    }
    return neighbours;
}

double OcGrid::indexToX(int index) {
    return ocgrid->info.origin.position.x + (0.5 + static_cast<double>(colFromIndex(index)))*ocgrid->info.resolution;
}

double OcGrid::indexToY(int index) {
    return ocgrid->info.origin.position.y + (0.5 + static_cast<double>(rowFromIndex(index)))*ocgrid->info.resolution;
}

int OcGrid::posToIndex(double x, double y) {
    int col = std::floor((x-ocgrid->info.origin.position.x)/ocgrid->info.resolution);
    int row = std::floor((y-ocgrid->info.origin.position.y)/ocgrid->info.resolution);
    return indexFromRowCol(row, col);
}

bool OcGrid::neighbours(int cell_1, int cell_2, bool diagonal) {
    std::vector<int> neighbours = getNeighbours(cell_1, diagonal);
    for(auto n:neighbours) if(n == cell_2) return true;
    return false;
}

int OcGrid::upIndex(int index) {
    if(inGrid(rowFromIndex(index)+1, colFromIndex(index))) return indexFromRowCol(rowFromIndex(index)+1, colFromIndex(index));
    return -1;
}

int OcGrid::downIndex(int index) {
    if(inGrid(rowFromIndex(index)-1, colFromIndex(index))) return indexFromRowCol(rowFromIndex(index)-1, colFromIndex(index));
    return -1;
}

int OcGrid::leftIndex(int index) {
    if(inGrid(rowFromIndex(index), colFromIndex(index)-1)) return indexFromRowCol(rowFromIndex(index), colFromIndex(index)-1);
    return -1;
}

int OcGrid::rightIndex(int index) {
    if(inGrid(rowFromIndex(index), colFromIndex(index)+1)) return indexFromRowCol(rowFromIndex(index), colFromIndex(index)+1);
    return -1;
}











