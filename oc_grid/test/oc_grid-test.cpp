#include <gtest/gtest.h>
#include <oc_grid/oc_grid.h>
#include <ros/package.h>
#include <iostream>


// Declare a test
TEST(OcGrid, FileConstructor)
{

    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/turn_test.yaml");
    OcGrid map(path);

    ASSERT_EQ(map.oc_grid.info.height, 10);
    ASSERT_EQ(map.oc_grid.info.width,  10);
    ASSERT_TRUE(fabs(map.oc_grid.info.resolution - 0.2) < 0.000001);
    ASSERT_DOUBLE_EQ(map.oc_grid.info.origin.position.x, -1.0);
    ASSERT_DOUBLE_EQ(map.oc_grid.info.origin.position.y, -1.0);

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

    for(unsigned i = 0; i < map.oc_grid.data.size(); i++) ASSERT_EQ(static_cast<int>(map.oc_grid.data[i]), correct_data[i]);
}

TEST(OcGrid, ImageExport)
{

    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/small_test.yaml");
    OcGrid map(path);
    cv::Mat image;
    map.exportMapImage(image);

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

TEST(OcGrid, unknownToOccupied)
{

    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/small_test.yaml");
    OcGrid map(path);
    map.unknownToOccupied();

    std::vector<int> correct_data = {0,0,0,
                                     100,100,100,
                                     100,100,100};

    for(unsigned i = 0; i < map.oc_grid.data.size(); i++) ASSERT_EQ(static_cast<int>(map.oc_grid.data[i]), correct_data[i]);
}

TEST(OcGrid, gradientInflat)
{

    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/turn_test.yaml");
    OcGrid map(path);
    map.unknownToOccupied();
    map.gradientInflate(true);

    std::vector<int> correct_data = {100,100,100,100,100,100,75,50,25,0,
                                     100,100,100,100,100,75,75,50,25,0,
                                     75,75,75,75,75,75,50,50,25,0,
                                     50,50,50,50,50,50,50,25,25,25,
                                     25,25,25,25,25,25,25,50,50,50,
                                     25,25,25,25,25,25,50,50,75,75,
                                     50,50,50,50,50,50,50,75,75,100,
                                     75,75,75,75,75,75,75,75,100,100,
                                     100,100,100,100,100,100,100,100,100,100,
                                     100,100,100,100,100,100,100,100,100,100};

    for(unsigned i = 0; i < map.gradient_grid.size(); i++) ASSERT_EQ(static_cast<int>(map.gradient_grid[i]), correct_data[i]);

    map.gradientInflate(false);

    correct_data = {100,100,100,100,100,100,80,60,40,20,
                    100,100,100,100,100,80,60,40,20,0,
                    80,80,80,80,80,60,40,20,0,20,
                    60,60,60,60,60,40,20,0,20,40,
                    40,40,40,40,40,20,20,20,40,60,
                    40,40,40,40,40,40,40,40,60,80,
                    60,60,60,60,60,60,60,60,80,100,
                    80,80,80,80,80,80,80,80,100,100,
                    100,100,100,100,100,100,100,100,100,100,
                    100,100,100,100,100,100,100,100,100,100};
    for(unsigned i = 0; i < map.gradient_grid.size(); i++) ASSERT_EQ(static_cast<int>(map.gradient_grid[i]), correct_data[i]);
}

TEST(OcGrid, cluster)
{
    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/turn_test.yaml");
    OcGrid map(path);
    map.unknownToOccupied();
    map.clusterOccupied();

    std::vector<int> correct_data = {1,1,1,1,1,1 ,0  ,0  ,0  ,0  ,
                                     1,1,1,1,1,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,2,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,2,2,
                                     2,2,2,2,2,2,2,2,2,2,
                                     2,2,2,2,2,2,2,2,2,2};

    for(unsigned i = 0; i < map.cluster_grid.size(); i++) {
        ASSERT_EQ(static_cast<int>(map.cluster_grid[i]), correct_data[i]);
    }
}

TEST(OcGrid, path)
{
    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/multiple_path_test.yaml");
    OcGrid map(path);
    std::vector<int> map_path, end_points;
    map.unknownToOccupied();
    map.clusterOccupied();
    map.inflate(true, true);
    map.oc_grid.data = map.inflated_grid;


    cv::Mat im;
    map.exportMapImage(im);
    std::string path2 = ros::package::getPath("oc_grid");
    path2.append("/im.png");
    cv::imwrite(path2, im);

    std::vector<int> correct_data = {1,1,1,1,1,1 ,0  ,0  ,0  ,0  ,
                                     1,1,1,1,1,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,2,
                                     0  ,0  ,0  ,0  ,0  ,0  ,0  ,0  ,2,2,
                                     2,2,2,2,2,2,2,2,2,2,
                                     2,2,2,2,2,2,2,2,2,2};

    ASSERT_EQ(1,1);
}

TEST(OcGrid, inGrid) {
    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/small_test.yaml");
    OcGrid map(path);

    ASSERT_TRUE(map.inGrid(4));
    ASSERT_TRUE(map.inGrid(1,1));

    ASSERT_FALSE(map.inGrid(-1));
    ASSERT_FALSE(map.inGrid(9));
    ASSERT_FALSE(map.inGrid(3,3));
    ASSERT_FALSE(map.inGrid(-1,0));
}

TEST(OcGrid, index_row_col_conversions)
{
    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/small_test.yaml");
    OcGrid map(path);

    ASSERT_EQ(map.indexFromRowCol(1,1), 4);
    ASSERT_EQ(map.rowFromIndex(4), 1);
    ASSERT_EQ(map.colFromIndex(4), 1);
}

TEST(OcGrid, getNeighbours) {
    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/small_test.yaml");
    OcGrid map(path);

    std::vector<int> n_1 = map.getNeighbours(4, true);
    std::vector<int> n_2 = map.getNeighbours(0, false);

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
    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/small_test.yaml");
    OcGrid map(path);

    ASSERT_TRUE(map.neighbours(4,1));
    ASSERT_TRUE(map.neighbours(4,3));
    ASSERT_TRUE(map.neighbours(4,5));
    ASSERT_TRUE(map.neighbours(4,7));
    ASSERT_TRUE(map.neighbours(1,4));
    ASSERT_TRUE(map.neighbours(3,4));
    ASSERT_TRUE(map.neighbours(5,4));
    ASSERT_TRUE(map.neighbours(7,4));
    ASSERT_TRUE(map.neighbours(4,0));
    ASSERT_TRUE(map.neighbours(4,2));
    ASSERT_TRUE(map.neighbours(4,6));
    ASSERT_TRUE(map.neighbours(4,8));
    ASSERT_TRUE(map.neighbours(0,4));
    ASSERT_TRUE(map.neighbours(2,4));
    ASSERT_TRUE(map.neighbours(6,4));
    ASSERT_TRUE(map.neighbours(8,4));

    ASSERT_FALSE(map.neighbours(4,0, false));
    ASSERT_FALSE(map.neighbours(4,2, false));
    ASSERT_FALSE(map.neighbours(4,6, false));
    ASSERT_FALSE(map.neighbours(4,8, false));
    ASSERT_FALSE(map.neighbours(0,4, false));
    ASSERT_FALSE(map.neighbours(2,4, false));
    ASSERT_FALSE(map.neighbours(6,4, false));
    ASSERT_FALSE(map.neighbours(8,4, false));

    ASSERT_TRUE(map.neighbours(6,3));
    ASSERT_TRUE(map.neighbours(1,5));
    ASSERT_TRUE(map.neighbours(7,8));

    ASSERT_FALSE(map.neighbours(6,2));
    ASSERT_FALSE(map.neighbours(6,1));
    ASSERT_FALSE(map.neighbours(6,0));
}

TEST(OcGrid, directionIndex) {
    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/small_test.yaml");
    OcGrid map(path);

    ASSERT_EQ(map.upIndex(4), 7);
    ASSERT_EQ(map.downIndex(4), 1);
    ASSERT_EQ(map.leftIndex(4), 3);
    ASSERT_EQ(map.rightIndex(4), 5);

    ASSERT_EQ(map.upIndex(7), -1);
    ASSERT_EQ(map.downIndex(1), -1);
    ASSERT_EQ(map.leftIndex(3), -1);
    ASSERT_EQ(map.rightIndex(5), -1);
}

TEST(OcGrid, outerIndex) {
    std::string path = ros::package::getPath("oc_grid");
    path.append("/maps/small_test.yaml");
    OcGrid map(path);

    std::vector<int> indexs = map.getOuterIndexs();
    std::vector<int> correct_n_1 = {0,1,2,5,8,7,6,3};

    ASSERT_EQ(indexs.size(), 8);
    for(unsigned i = 0; i < indexs.size(); i++) {
        ASSERT_EQ(indexs[i], correct_n_1[i]);
    }

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
