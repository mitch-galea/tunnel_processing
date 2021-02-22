#include <tunnel_centering/tunnel_path.h>

TunnelPath::TunnelPath(bool d_inflate, bool d_path)
:d_inflate_(d_inflate), d_path_(d_path)
{}

std::vector<int> TunnelPath::computePath(OcGrid &oc_grid) {
    // sets unknown cells to occupied
    oc_grid.unknownToOccupied();
    // sets up the gradient grid used for dijkstra cost
    oc_grid.gradientInflate(d_inflate_);
    // clusters occupied areas of grid for path inflation
    oc_grid.clusterOccupied();
    // inflates to single thickness path
    oc_grid.inflate(d_inflate_, d_path_);

    // gets end points and classifies them as left or right
    std::vector<int> left_end_points, right_end_points;
    for(unsigned i = 0; i < oc_grid.inflated_grid.size(); i++) {
        if(oc_grid.inflated_grid[i] == 0 && ((oc_grid.rowFromIndex(i) == 0 || oc_grid.rowFromIndex(i) == oc_grid.oc_grid.info.height -1) ||
        (oc_grid.colFromIndex(i) == 0 || oc_grid.colFromIndex(i) == oc_grid.oc_grid.info.width -1))) {
            if(oc_grid.colFromIndex(i) < oc_grid.oc_grid.info.width/2) {
                left_end_points.push_back(i);
            } else right_end_points.push_back(i);
        }
    }

    // check to ensure at least 1 left and right end point
    if(left_end_points.empty() || right_end_points.empty()) ROS_ERROR("TunnelPath computePath: no left or right end points");

    std::vector<int> path, temp_path;
    int temp_cost;
    int min_cost = 1000000;

    for(auto l:left_end_points) {
        DijkstraGrid dijkstra_grid(oc_grid, l, d_path_);
        for(auto r:right_end_points) {
            temp_cost = dijkstra_grid.path(temp_path, r);
            if(temp_cost < min_cost ) {
                min_cost = temp_cost;
                path = temp_path;
            }
        }
    }

    return path;

}

std::vector<int> TunnelPath::smoothPath(OcGrid &oc_grid, std::vector<int> path) {
    std::vector<DoubleCell> double_path;
    // creates a path using double cell
    for(auto i:path) {
        DoubleCell cell;
        cell.x = static_cast<double>(oc_grid.colFromIndex(i));
        cell.y = static_cast<double>(oc_grid.rowFromIndex(i));
        double_path.push_back(cell);
    }
    std::vector<DoubleCell> new_double_path = double_path;
    double tolerance = 0.00000001;
    double weight_data = 0.5, weight_smooth = 0.1;

    // smooths path with gradient descent
    double change = tolerance;
    while(change >= tolerance) {

        change = 0.0;
        double prev_x = 0.0, prev_y = 0.0;
        for(unsigned i = 1; i < path.size() - 1; i++) {
            prev_x = new_double_path[i].x;
            prev_y = new_double_path[i].y;
            new_double_path[i].x += weight_data * (double_path[i].x - new_double_path[i].x) +
                                   weight_smooth * (new_double_path[i-1].x + new_double_path[i+1].x - 2.0 * new_double_path[i].x);
            new_double_path[i].y += weight_data * (double_path[i].y - new_double_path[i].y) +
                                   weight_smooth * (new_double_path[i-1].y + new_double_path[i+1].y - 2.0 * new_double_path[i].y);
            change += fabs(prev_x - new_double_path[i].x);
            change += fabs(prev_y - new_double_path[i].y);
        }
    }

    // tranfers path back to discrete indexs
    std::vector<int> new_path;
    for(auto cell:new_double_path) {
        new_path.push_back(oc_grid.indexFromRowCol(floor(cell.y), floor(cell.x)));
    }
    return new_path;
}

void TunnelPath::setParams(bool d_inflate, bool d_path) {
    d_inflate_ = d_inflate;
    d_path_ = d_path;
}


