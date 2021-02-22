#include <tunnel_centering/dijkstra_grid.h>



struct CompareCost {
    bool operator()(Cell const& c1, Cell const& c2)
    {
        return c1.cost > c2.cost;
    }
};

DijkstraGrid::DijkstraGrid(OcGrid oc_grid, int start_index, bool diagonal) {
    dijkstra(oc_grid, start_index, diagonal);
}

void DijkstraGrid::dijkstra(OcGrid oc_grid, int start_index, bool diagonal) {
    start_index_ = start_index;
    dijkstra_grid_.resize(oc_grid.oc_grid.data.size());

    // set up dijkstra grid data structure
    for(unsigned i = 0; i < oc_grid.oc_grid.data.size(); i++) {
        dijkstra_grid_[i].index = i;
        dijkstra_grid_[i].gradient = oc_grid.gradient_grid[i];
        dijkstra_grid_[i].cost = INIT_VALUE;
    }
    // initialise start index cost as zero
    dijkstra_grid_[start_index].cost = dijkstra_grid_[start_index].gradient;

    std::priority_queue<Cell, std::vector<Cell>, CompareCost> opened_cells;
    opened_cells.push(dijkstra_grid_[start_index]);

    std::set<int> closed_cells;

    Cell current_cell;
    while(!opened_cells.empty()) {
        // pops front cell
        current_cell = opened_cells.top();
        opened_cells.pop();
        // checks if cell is not already closed
        if(closed_cells.find(current_cell.index) == closed_cells.end()) {
            std::vector<int> neighbours = oc_grid.getNeighbours(current_cell.index, diagonal);
            for(auto n:neighbours) {
                if(oc_grid.inflated_grid[n] != 100) {
                    int t_cost = current_cell.cost + dijkstra_grid_[n].gradient;
                    if(t_cost < dijkstra_grid_[n].cost) {
                        dijkstra_grid_[n].cost = t_cost;
                        dijkstra_grid_[n].parent = current_cell.index;
                        opened_cells.push(dijkstra_grid_[n]);
                    }
                }
            }
        }
        // adds cell to closed cells
        closed_cells.insert(current_cell.index);
    }


}

int DijkstraGrid::path(std::vector<int> &path, int end_index) {
    path.clear();

    if(dijkstra_grid_[end_index].cost == INIT_VALUE) {
        ROS_ERROR("Dijkstra search: no path found");
        return -1;
    }
    else {
        Cell current_cell = dijkstra_grid_[end_index];
        path.push_back(current_cell.index);
        while(current_cell.index != start_index_) {
            current_cell = dijkstra_grid_[current_cell.parent];
            path.push_back(current_cell.index);
        }
        std::reverse(path.begin(), path.end());
        return dijkstra_grid_[end_index].cost;
    }
}
