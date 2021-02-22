#ifndef SRC_TUNNEL_PATH_H
#define SRC_TUNNEL_PATH_H

#include <oc_grid/oc_grid.h>
#include <ros/ros.h>
#include <tunnel_centering/dijkstra_grid.h>

struct DoubleCell {
    double x;
    double y;
};

class TunnelPath {
private:
    bool d_inflate_;
    bool d_path_;
public:
    TunnelPath(bool d_inflate, bool d_path);
    std::vector<int> computePath(OcGrid &oc_grid);
    std::vector<int> smoothPath(OcGrid &oc_grid, std::vector<int> path);

    void setParams(bool d_inflate, bool d_path);
};

#endif
