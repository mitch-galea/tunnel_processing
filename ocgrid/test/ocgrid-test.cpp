#include <gtest/gtest.h>
#include <ocgrid/ocgrid.hpp>
#include <ros/package.h>
#include <iostream>


// Declare a test
TEST(OcGrid, Generator)
{

    std::string path = ros::package::getPath("ocgrid");
    path.append("/maps/turn_test.yaml");

    nav_msgs::OccupancyGrid ocgrid = Ocgrid::generateOcgrid(path);

    ASSERT_EQ(ocgrid.info.height, 10);
    ASSERT_EQ(ocgrid.info.width,  10);
    ASSERT_TRUE(fabs(ocgrid.info.resolution - 0.2) < 0.000001);
    ASSERT_DOUBLE_EQ(ocgrid.info.origin.position.x, -1.0);
    ASSERT_DOUBLE_EQ(ocgrid.info.origin.position.y, -1.0);

    std::vector<int> correct_data = {100,100,100,100,100,-1 ,0  ,0  ,0  ,0  ,
                                     100,100,100,100,100,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,100,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,100,100,
                                     100,100,100,100,100,100,100,100,100,100,
                                     100,100,100,100,100,100,100,100,100,100};

    for(unsigned i = 0; i < ocgrid.data.size(); i++) ASSERT_EQ(static_cast<int>(ocgrid.data[i]), correct_data[i]);
}

TEST(OcGrid, ImageExport)
{

    std::string path = ros::package::getPath("ocgrid");
    path.append("/maps/small_test.yaml");
    nav_msgs::OccupancyGrid ocgrid = Ocgrid::generateOcgrid(path);
    cv::Mat image;
    Ocgrid::exportMapImage(ocgrid, image);

    std::vector<cv::Vec3b> correct_image = {{0,0,0},{0,0,0},{0,0,0},
                                            {127,127,127},{127,127,127},{127,127,127},
                                            {255,255,255},{255,255,255},{255,255,255}};

    for(unsigned row = 0; row < image.rows; row++) {
        for(unsigned col = 0; col < image.cols; col++) {
            ASSERT_EQ(image.at<cv::Vec3b>(row, col)[0], correct_image[row*image.cols + col][0]);
            ASSERT_EQ(image.at<cv::Vec3b>(row, col)[1], correct_image[row*image.cols + col][1]);
            ASSERT_EQ(image.at<cv::Vec3b>(row, col)[2], correct_image[row*image.cols + col][2]);
        }
    }
}

TEST(OcGrid, inGrid) {
    std::string path = ros::package::getPath("ocgrid");
    path.append("/maps/small_test.yaml");
    nav_msgs::OccupancyGrid ocgrid = Ocgrid::generateOcgrid(path);

    ASSERT_TRUE(Ocgrid::inGrid(ocgrid,4));
    ASSERT_TRUE(Ocgrid::inGrid(ocgrid,1,1));

    ASSERT_FALSE(Ocgrid::inGrid(ocgrid,-1));
    ASSERT_FALSE(Ocgrid::inGrid(ocgrid,9));
    ASSERT_FALSE(Ocgrid::inGrid(ocgrid,3,3));
    ASSERT_FALSE(Ocgrid::inGrid(ocgrid,-1,0));
}

TEST(OcGrid, index_row_col_conversions)
{
    std::string path = ros::package::getPath("ocgrid");
    path.append("/maps/small_test.yaml");
    nav_msgs::OccupancyGrid ocgrid = Ocgrid::generateOcgrid(path);

    ASSERT_EQ(Ocgrid::indexFromRowCol(ocgrid, 1,1), 4);
    ASSERT_EQ(Ocgrid::rowFromIndex(ocgrid, 4), 1);
    ASSERT_EQ(Ocgrid::colFromIndex(ocgrid,4), 1);
}

TEST(OcGrid, getNeighbours) {
    std::string path = ros::package::getPath("ocgrid");
    path.append("/maps/small_test.yaml");
    nav_msgs::OccupancyGrid ocgrid = Ocgrid::generateOcgrid(path);

    std::vector<int> n_1 = Ocgrid::getNeighbours(ocgrid, 4, true);
    std::vector<int> n_2 = Ocgrid::getNeighbours(ocgrid, 0, false);

    std::vector<int> correct_n_1 = {1,3,5,7,0,2,6,8};
    std::vector<int> correct_n_2 = {1,3};

    ASSERT_EQ(n_1.size(), 8);
    ASSERT_EQ(n_2.size(), 2);

    for(unsigned i = 0; i < n_1.size(); i++) {
        ASSERT_EQ(n_1[i], correct_n_1[i]);
    }
    for(unsigned i = 0; i < n_2.size(); i++) {
        ASSERT_EQ(n_2[i], correct_n_2[i]);
    }
}

TEST(OcGrid, neighbours) {
    std::string path = ros::package::getPath("ocgrid");
    path.append("/maps/small_test.yaml");
    nav_msgs::OccupancyGrid ocgrid = Ocgrid::generateOcgrid(path);

    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 4,1));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 4,3));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 4,5));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 4,7));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 1,4));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 3,4));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 5,4));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 7,4));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 4,0));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 4,2));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 4,6));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 4,8));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 0,4));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 2,4));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 6,4));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 8,4));

    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 4,0, false));
    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 4,2, false));
    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 4,6, false));
    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 4,8, false));
    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 0,4, false));
    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 2,4, false));
    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 6,4, false));
    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 8,4, false));

    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 6,3));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 1,5));
    ASSERT_TRUE(Ocgrid::isNeighbours(ocgrid, 7,8));

    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 6,2));
    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 6,1));
    ASSERT_FALSE(Ocgrid::isNeighbours(ocgrid, 6,0));
}

TEST(OcGrid, directionIndex) {
    std::string path = ros::package::getPath("ocgrid");
    path.append("/maps/small_test.yaml");
    nav_msgs::OccupancyGrid ocgrid = Ocgrid::generateOcgrid(path);

    ASSERT_EQ(Ocgrid::upIndex(ocgrid, 4), 7);
    ASSERT_EQ(Ocgrid::downIndex(ocgrid, 4), 1);
    ASSERT_EQ(Ocgrid::leftIndex(ocgrid, 4), 3);
    ASSERT_EQ(Ocgrid::rightIndex(ocgrid, 4), 5);

    ASSERT_EQ(Ocgrid::upIndex(ocgrid, 7), -1);
    ASSERT_EQ(Ocgrid::downIndex(ocgrid, 1), -1);
    ASSERT_EQ(Ocgrid::leftIndex(ocgrid, 3), -1);
    ASSERT_EQ(Ocgrid::rightIndex(ocgrid, 5), -1);
}

TEST(OcGrid, transforms) {
    std::string path = ros::package::getPath("ocgrid");
    path.append("/maps/small_test.yaml");
    nav_msgs::OccupancyGrid ocgrid = Ocgrid::generateOcgrid(path);

    ASSERT_DOUBLE_EQ(Ocgrid::indexToX(ocgrid, 4), 2.6);
    ASSERT_DOUBLE_EQ(Ocgrid::indexToX(ocgrid, 0), 1.6);
    ASSERT_DOUBLE_EQ(Ocgrid::indexToY(ocgrid, 4), 1.0);
    ASSERT_DOUBLE_EQ(Ocgrid::indexToY(ocgrid, 0), 0.0);

    ASSERT_EQ(Ocgrid::posToIndex(ocgrid, 1.6, -0.1), 0);
    ASSERT_EQ(Ocgrid::posToIndex(ocgrid, 2.6, 1.0), 4);
    
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
