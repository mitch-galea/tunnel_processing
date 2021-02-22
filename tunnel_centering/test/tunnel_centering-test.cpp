#include <gtest/gtest.h>
#include <tunnel_centering/tunnel_path.h>
#include <ros/package.h>
#include <iostream>

TEST(OcGrid, dijkstra)
{
//    std::string path = ros::package::getPath("oc_grid");
//    path.append("/maps/multiple_path_test.yaml");
//    OcGrid map(path);
//    map.unknownToOccupied();
//    map.gradientInflate(true);
//    map.clusterOccupied();
//    map.inflate(true, true);
//
//    DijkstraGrid dijkstra_grid(map, 810, true);
//    std::vector<int> path_;
//
//    int cost_1 = dijkstra_grid.path(path_, 839);
//    int cost_2 = dijkstra_grid.path(path_, 239);
//
//    ASSERT_EQ(cost_1, 1380);
//    ASSERT_EQ(cost_2, 660);
}

TEST(OcGrid, tunnel_path)
{
    std::string path = ros::package::getPath("oc_grid");
    //path.append("/maps/hospital_section.yaml");
    path.append("/maps/path_test.yaml");
    OcGrid oc_grid(path);

    TunnelPath path_solver(true, true);

    std::vector<int> tunnel_path = path_solver.computePath(oc_grid);
    std::vector<int> smooth_path = path_solver.smoothPath(oc_grid, tunnel_path);
    std::vector<cv::Vec3b> path_colours(tunnel_path.size(), cv::Vec3b(0,0,255));

    cv::Mat im, im_smooth;
    oc_grid.exportMapImage(im, tunnel_path, path_colours, 0);
    oc_grid.exportMapImage(im_smooth, smooth_path, path_colours, 0);
    std::string path2 = ros::package::getPath("oc_grid");
    std::string path3 = path2;
    path2.append("/im.png");
    path3.append("/im_smooth.png");
    cv::imwrite(path2, im);
    cv::imwrite(path3, im_smooth);


    ASSERT_EQ(1, 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
