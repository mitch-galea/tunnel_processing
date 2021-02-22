#ifndef SRC_DIJKSTRAGRID_H
#define SRC_DIJKSTRAGRID_H

#include <oc_grid/oc_grid.h>
#include <ros/ros.h>
#include <queue>
#include <algorithm>
#include <set>

const int INIT_VALUE = 1000000;

struct Cell {
    int index;
    int gradient;
    int cost;
    int parent;
};

class DijkstraGrid {
private:
    std::vector<Cell> dijkstra_grid_;
    int start_index_;
public:
    DijkstraGrid(OcGrid oc_grid, int start_index, bool diagonal);
    void dijkstra(OcGrid oc_grid, int start_index, bool diagonal);
    int path(std::vector<int> &path, int end_index);
};


#endif //SRC_DIJKSTRAGRID_H
