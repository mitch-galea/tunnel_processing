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
    /// ROS Occupancy Grid subscriber
    ros::Subscriber ocgridSub_;
    /// ROS GridCell publiser
    ros::Publisher gridCellPub_;

    
public:
    TunnelPathNode(ros::NodeHandle nh)
        :nh_(nh)
    {
        /// nodehandle for parameters
        ros::NodeHandle pn("~");
        /// strings for topics
        std::string ocgridTopic, gridCellTopic;
        pn.param<std::string>("ocgridTopic", ocgridTopic, "/ocgrid");
        pn.param<std::string>("gridCellTopic", gridCellTopic, "/path");
        /// initialises subscriber and publisher
        ocgridSub_ = nh_.subscribe<nav_msgs::OccupancyGrid> (ocgridTopic, 1, &TunnelPathNode::ocgridCallback, this);
        gridCellPub_ = nh_.advertise<nav_msgs::GridCells> (gridCellTopic, 1);

        // /// Sets up dynamic reconfigure server
        // static dynamic_reconfigure::Server<pointcloud_to_ocgrid::settingsConfig> config_server;
        // dynamic_reconfigure::Server<pointcloud_to_ocgrid::settingsConfig>::CallbackType config_callType;
        // config_callType = boost::bind(&PointcloudToOcgridNode::updateConfig, this, _1, _2);
        // config_server.setCallback(config_callType);

    }

    void ocgridCallback(const nav_msgs::OccupancyGridConstPtr& ocgridMsg) {
        /// Computes path
        nav_msgs::OccupancyGrid ocgrid = *ocgridMsg;
        nav_msgs::GridCells gridCells = TunnelPath::computeTunnelPath(ocgrid);
        gridCells.header.frame_id = "velodyne";
        /// Published path
        gridCellPub_.publish(gridCells);
    }

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