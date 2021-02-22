/** @pointcloud-to-ocgrid-node.cpp
 *
 * Node for pointcloud-to-ocgrid conversion
 */

#pragma once

#include <ros/ros.h>

#include "pointcloud-to-ocgrid/pointcloud-to-ocgrid.hpp"

class PointcloudToOcgridNode
{
private:
    ros::NodeHandle nh_;
public:
    PointcloudToOcgridNode(ros::NodeHandle nh)
        :nh_(nh)
    {
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud-to-ocgrid-node");

    ros::NodeHandle nh;

    ros::spin();

  return 0;
}