/** @pointcloud-to-ocgrid-node.cpp
 *
 * Node for pointcloud-to-ocgrid conversion
 */

#include <ros/ros.h>

#include "pointcloud-to-ocgrid/pointcloud-to-ocgrid.hpp"

class PointcloudToOcgridNode
{
private:
    /// ROS Nodehandle
    ros::NodeHandle nh_;
    /// ROS Pointcloud subscriber
    ros::Subscriber pcSub_;
    /// ROS Occupancy Grid publisher
    ros::Publisher ocgridPub_;

    
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