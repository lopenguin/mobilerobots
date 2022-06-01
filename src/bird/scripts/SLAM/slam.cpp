/*
SLAM.h: ported version of slam.py
*/
#include <ros/ros.h>
#include <node_handle.h>

#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>


class SLAM() {
public:
    void SLAM() : m_leastSqs{}
    {
        // TODO: initialize map to odom transform

        // set up least squares
    }


    // Callbacks
    void cb_odom(const nav_msgs::Odometry::ConstPtr& msg);
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
    ros::NodeHandle m_nh;
    
    // Publishers
    ros::Publisher m_pub = nh.advertise<std_msgs::String>("topic_name", 5);

    // Transforms

    // Other helpers
    LeastSquares m_leastSqs;
    
};


class PlanarTransform() {
public:
    void PlanarTransform(double px, double py, double qz, double qw) {
        m_px = px;
        m_py = py;
        m_qz = qz;
        m_qw = qw;
    }

    // Processing
    void inParent(ox, oy) {
        
    }

    // getter functions
    double x() { return m_px; }
    double y() { return m_py; }
    double sin() { return 2.0*m_qz*m_qw; }
    double cos() { return m_qz*m_qz + m_qw*m_qw; }

private:
    double m_px{};
    double m_py{};
    double m_qz{};
    double m_qw{};
};