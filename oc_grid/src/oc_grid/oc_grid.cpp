#include "oc_grid/oc_grid.h"

OcGrid::OcGrid(nav_msgs::OccupancyGrid oc_grid_in): oc_grid(oc_grid_in)
{}

OcGrid::OcGrid(std::string map_file_path) {
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

        oc_grid = resp.map;
    }
}

void OcGrid::exportMapImage(cv::Mat &image, int mode) {
    image.create(oc_grid.info.height, oc_grid.info.width, CV_8UC3);
    for(unsigned row = 0; row < oc_grid.info.height; row++) {
        for(unsigned col = 0; col < oc_grid.info.width; col++) {
            cv::Vec3b value;
            int val;
            switch (mode) {
                case 0:
                    val = static_cast<int>(oc_grid.data[row*oc_grid.info.width + col]);
                    break;
                case 1:
                    val = static_cast<int>(gradient_grid[row*oc_grid.info.width + col]);
                    break;
                case 2:
                    val = static_cast<int>(cluster_grid[row*oc_grid.info.width + col]);
                    break;
                case 3:
                    val = static_cast<int>(inflated_grid[row*oc_grid.info.width + col]);
                    break;
                default:
                    val = static_cast<int>(oc_grid.data[row*oc_grid.info.width + col]);
            }
            if(val == -1) val = 127;
            else val = (100-val)*255/100;

            value[0] = val;
            value[1] = val;
            value[2] = val;

            unsigned im_row = oc_grid.info.height - 1 - row;
            image.at<cv::Vec3b>(im_row, col) = value;
        }
    }
}

void OcGrid::exportMapImage(cv::Mat &image, std::vector<int> grid_cells, std::vector<cv::Vec3b> colours, int mode) {
    exportMapImage(image, mode);
    for(unsigned i = 0; i < grid_cells.size(); i++) {
        int im_row = oc_grid.info.height - 1 - rowFromIndex(grid_cells[i]), im_col = colFromIndex(grid_cells[i]);
        image.at<cv::Vec3b>(im_row, im_col) = colours[i];
    }
}

void OcGrid::unknownToOccupied() {
    for(auto && cell:oc_grid.data) if(static_cast<int>(cell) != 0) cell = static_cast<int8_t>(100);
}

void OcGrid::gradientInflate(bool diagonal) {
    gradient_grid = oc_grid.data;
    int level = 100;
    bool changed = true;
    while(changed) {
        level --;
        changed = false;
        // iterates through each square
        for(int unsigned i = 0; i < gradient_grid.size(); i++) {
            if(gradient_grid[i] == level + 1) {
                std::vector<int> neighbours = getNeighbours(i, diagonal);
                for(auto n:neighbours) {
                    if(inGrid(n)) {
                        if(gradient_grid[n] == 0){
                            gradient_grid[n] = level;
                            changed = true;
                        }
                    }
                }
            }
        }
    }
    level++;
    int delta = 100 - level;
    for(auto && cell:gradient_grid) cell = (cell - level)*100/delta;
}

void OcGrid::clusterOccupied() {
    cluster_grid.resize(oc_grid.data.size());
    cluster_grid = oc_grid.data;
    int count = 0;
    for(unsigned i = 0; i < oc_grid.data.size(); i++) {
        if(cluster_grid[i] == 100) {
            count++;
            cluster_grid[i] = count;
            std::queue<int> cluster_queue;
            cluster_queue.push(i);
            int current_i;
            while (!cluster_queue.empty()) {
                current_i = cluster_queue.front();
                cluster_queue.pop();
                std::vector<int> neighbours = getNeighbours(current_i, true);
                for(auto n:neighbours) {
                    if(cluster_grid[n] == 100) {
                        cluster_grid[n] = count;
                        cluster_queue.push(n);
                    }
                }
            }
        }
    }
}

void OcGrid::inflate(bool d_inflate, bool d_path) {
    inflated_grid = oc_grid.data;
    std::queue<int> to_expand;
    for (unsigned i = 0; i < inflated_grid.size(); i++) if (inflated_grid[i] == 100) to_expand.push(i);
    while (!to_expand.empty()) {
        int index = to_expand.front();
        to_expand.pop();
        std::vector<int> exp_neighbours = getNeighbours(index, d_inflate);
        for (auto exp_n:exp_neighbours) {
            if (inflated_grid[exp_n] == 0) {
                bool cluster_hit = false;
                std::vector<int> path_neighbours = getNeighbours(exp_n, !d_path);
                for (auto path_n:path_neighbours) {
                    if ((inflated_grid[path_n] == 100) && cluster_grid[path_n] != cluster_grid[index]) {
                        cluster_hit = true;
                        break;
                    }
                }
                if (!cluster_hit) {
                    inflated_grid[exp_n] = 100;
                    cluster_grid[exp_n] = cluster_grid[index];
                    to_expand.push(exp_n);
                }
            }
        }
    }
}

std::vector<int> OcGrid::getOuterIndexs() {
    std::vector<int> outer_indexs;
    int i = 0;
    //bottom
    outer_indexs.push_back(i);
    while(i < oc_grid.info.width - 1) {
        i++;
        outer_indexs.push_back(i);
    }
    //right
    while(i < oc_grid.info.width*oc_grid.info.height-1) {
        i = i+oc_grid.info.width;
        outer_indexs.push_back(i);
    }
    //top
    while(i > oc_grid.info.width*(oc_grid.info.height-1)) {
        i--;
        outer_indexs.push_back(i);
    }
    //left
    while(i > oc_grid.info.width) {
        i = i-oc_grid.info.width;
        outer_indexs.push_back(i);
    }
    return outer_indexs;
}


bool OcGrid::inGrid(int index) {
    return index >= 0 && index < oc_grid.data.size();
}

bool OcGrid::inGrid(int row, int col) {
    return row >= 0 && row < oc_grid.info.height && col >= 0 && col < oc_grid.info.width;
}


int OcGrid::indexFromRowCol(int row, int col) {
    return row*oc_grid.info.width + col;
}

int OcGrid::rowFromIndex(int index) {
    return floor(static_cast<double>(index)/ static_cast<double>(oc_grid.info.width));
}

int OcGrid::colFromIndex(int index) {
    return index % oc_grid.info.width;
}

std::vector<int> OcGrid::getNeighbours(int index, bool diagonal) {
    if(index >= oc_grid.data.size()) ROS_ERROR("OcGrid getNeighbours: Index exceeds data");
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

geometry_msgs::Pose OcGrid::gridToPose(int index) {
    return geometry_msgs::Pose();
}

geometry_msgs::Pose OcGrid::gridToPose(int col, int row) {
    return geometry_msgs::Pose();
}

int OcGrid::poseToGrid(geometry_msgs::Pose) {
    return 0;
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











