/** @pointcloud-to-ocgrid-node.cpp
 *
 * Node for pointcloud-to-ocgrid conversion
 */

#include <ros/ros.h>

#include <tunnel_path/tunnel_path.hpp>
#include <ocgrid/ocgrid.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
//#include <tunnel_path/settingsConfig.h>

class TunnelPathNode
{
private:
    /// ROS Objects ///
    /// ROS Nodehandle
    ros::NodeHandle nh_;
    /// ROS Pointcloud subscriber
    //ros::Subscriber ocgridSub_;

    
public:
    TunnelPathNode(ros::NodeHandle nh)
        :nh_(nh)
    {
        // /// nodehandle for parameters
        // ros::NodeHandle pn("~");
        // /// strings for topics
        // std::string pcTopic, ocgridTopic;
        // pn.param<std::string>("pcTopic", pcTopic, "/velodyne_points");
        // pn.param<std::string>("ocgridTopic", ocgridTopic, "/ocgrid");
        // /// initialises subscriber and publisher
        // pcSub_ = nh_.subscribe<sensor_msgs::PointCloud2> (pcTopic, 1, &PointcloudToOcgridNode::pcCallback, this);
        // ocgridPub_ = nh_.advertise<nav_msgs::OccupancyGrid> (ocgridTopic, 5);

        // /// for debugging
        // pcPub_ = nh_.advertise<PointCloud> ("/points/filtered", 100);

        // /// Sets up dynamic reconfigure server
        // static dynamic_reconfigure::Server<pointcloud_to_ocgrid::settingsConfig> config_server;
        // dynamic_reconfigure::Server<pointcloud_to_ocgrid::settingsConfig>::CallbackType config_callType;
        // config_callType = boost::bind(&PointcloudToOcgridNode::updateConfig, this, _1, _2);
        // config_server.setCallback(config_callType);

    }

    // void pcCallback(const sensor_msgs::PointCloud2ConstPtr& pcMsg) {
    //     /// declaration of pointcloud for processing
    //     PointCloud::Ptr cloud (new PointCloud);
    //     //ros msg to pcl object
    //     pcl::fromROSMsg (*pcMsg, *cloud);

    //     //z filter
    //     pcl::PassThrough<PointType> zPassthroughFilter;
    //     zPassthroughFilter.setInputCloud (cloud);
    //     zPassthroughFilter.setFilterFieldName ("z");
    //     zPassthroughFilter.setFilterLimits (pointcloudZMin_, pointcloudZMax_);
    //     zPassthroughFilter.filter (*cloud);    

    //     pcPub_.publish(*cloud);    

    //     PointcloudToOcgrid::convertPointcloudToOcgrid(cloud, ocgrid_, inflation_, minClusterSize_);

    //     ocgridPub_.publish(ocgrid_);    
    // }

    // void updateConfig(pointcloud_to_ocgrid::settingsConfig &config, uint32_t level) {
    //     /// sets ocgrid info
    //     ocgrid_ = Ocgrid::generateOcgrid(config.ocgridWidth, config.ocgridHeight, config.ocgridResolution,
    //                                     config.ocgridXOrigin, config.ocgridYOrigin, 0);
    //     ocgrid_.header.frame_id = "velodyne";
    //     /// sets point cloud filter
    //     pointcloudZMin_ = config.pointcloudZMin;
    //     pointcloudZMax_ = config.pointcloudZMax;
    //     inflation_ = config.inflation;
    //     minClusterSize_ = config.minClusterSize;
    // }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tunnel_path_node");

    ros::NodeHandle nh;

    TunnelPathNode node(nh);

    ros::spin();

  return 0;
}