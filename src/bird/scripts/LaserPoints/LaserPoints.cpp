/*
Reads laserscan data and converts it into a point cloud in odometry frame.

Publish:    /scan_points            sensor_msgs/PointCloud2

Subscribe:  /scan                   sensor_msgs/LaserScan
            TF laser -> odom        geometry_msgs/TransformStamped
            /odom                   geometry_msgs/TransJointState
*/
#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <sstream>

class LaserPoints {
public:
    LaserPoints() {
        m_pubScanPoints = m_nh.advertise<sensor_msgs::PointCloud2>("/scan_points", 2);

        // create subscribers
        m_subScan = m_nh.subscribe("/scan", 1, &LaserPoints::cb_scan, this);
        ROS_INFO("Class initialized.");
    }


    // callback
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& scan) {
        if(!m_listener.waitForTransform(scan->header.frame_id, "/base",
            scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
            ros::Duration(0.1))) 
        { return; }
        sensor_msgs::PointCloud2 cloud;
        m_projector.transformLaserScanToPointCloud("base", *scan, cloud, m_listener);

        // publish the point cloud
        m_pubScanPoints.publish(cloud);
    }

private:
    ros::NodeHandle m_nh;

    laser_geometry::LaserProjection m_projector;
    tf::TransformListener m_listener;

    ros::Publisher m_pubScanPoints;
    ros::Subscriber m_subScan;
};


int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "laser_converter");

    LaserPoints lpts;

    ros::spin();
    return 0;
}