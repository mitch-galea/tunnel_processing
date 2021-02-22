#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

TEST(TESTSuite, addTwoInts)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::vector<std::vector<int>> test_data = {{0.1}}
    EXPECT_EQ(1, 1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
