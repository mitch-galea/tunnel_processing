#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <oc-grid/oc-grid.h>
#include <vector>

#include "pointcloud-to-ocgrid/pointcloud-to-ocgrid.hpp"

TEST(TESTSuite, PointcloudToOcgridTest)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    /// Generates the test data
    std::vector<std::vector<double>> test_data = {{-0.4, 0.8},
                                                  {0.75, 0.75},
                                                  {1.1, 0.99999},
                                                  {0.8, 0.3},
                                                  {-0.01, 0.01},
                                                  {1.49, 0.1},
                                                  {0.1, -0.1},
                                                  {1.1, -0.3},
                                                  {0.1, -0.6},
                                                  {0.3, -0.6},
                                                  {0.8, -0.6},
                                                  {1.1, -0.6},
                                                  {-0.3, -0.8},
                                                  {0.01, -0.8},
                                                  {0.26, -0.8},
                                                  {0.6, -0.8},
                                                  {0.8, -0.8},
                                                  {1.1, -0.8},
                                                  {1.49, -0.8},
                                                  {-0.1, 1.0001},
                                                  {1.5, 1.25},
                                                  {1.3, -1.0001},
                                                  {-0.500001, -0.8}};

    for (auto tp : test_data)
    {
        /// for each test data point, generate a point cloud point and add to cloud, only
        /// worry about x and y values as that is all that is used in PointCloudtoOcgrid Function
        pcl::PointXYZI p;
        p.x = tp[0];
        p.y = tp[1];
        cloud->points.push_back(p);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    /// Set up test input occupancy grid
    nav_msgs::OccupancyGrid::Ptr ocgrid(new nav_msgs::OccupancyGrid);
    ocgrid->info.width = 8;
    ocgrid->info.height = 8;
    ocgrid->info.resolution = 0.25;
    ocgrid->info.origin.position.x = -0.5;
    ocgrid->info.origin.position.y = -1.0;
    ocgrid->data.resize(ocgrid->info.width*ocgrid->info.height);

    /// Declares converter object and converts pointcloud to ocgrid
    PointcloudToOcgrid::convertPointcloudToOcgrid(cloud, ocgrid);

    std::string path = "/home/mitchellgalea/testim.png";

    OcGrid o(ocgrid);

    // Expected output
    const unsigned U = 0, O = 100;
    std::vector<unsigned> expected_output = {O, U, O, O, O, O, O, O,
                                             U, U, O, O, U, O, O, U,
                                             U, U, U, U, U, U, O, U,
                                             U, U, O, U, U, U, U, U,
                                             U, O, U, U, U, U, U, O,
                                             U, U, U, U, U, O, U, U,
                                             U, U, U, U, U, U, U, U,
                                             O, U, U, U, U, O, O, U};

    // Tests
    ASSERT_EQ(ocgrid->data.size(), 64);
    for (unsigned i = 0; i < expected_output.size(); i++)
    {
        ASSERT_EQ(ocgrid->data[i], expected_output[i]);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
