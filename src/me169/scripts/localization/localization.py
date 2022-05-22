#!/usr/bin/env python3
#
#   localization.py
#
#   Localization node.  This
#   (a) Converts from odom to pose reference
#
#   Node:       /localization
#   Publish:    /pose                   geometry_msgs/PoseStamped
#   Subscribe:  /odom                geometry_msgs/TransJointState
#               /scan                   sensor_msgs/LaserScan

import rospy
import math
import numpy as np
import tf2_ros
from PlanarTransform import PlanarTransform

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

from maputilities import cartesiantopixel, map_to_nearest_wall, pixeltocartesian

# Constants
UPDATE_FRACTION_MOVING_FAST = 1/100 # How much of a Delta to use per update
UPDATE_FRACTION_MOVING_SLOW = 1/60
UPDATE_FRACTION_STOPPED = 1/40

class LocalizationObj():
    def __init__(self):
        self.map_to_odom_TF = PlanarTransform.unity()


        ## initalize node
        # Create /pose publisher
        self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)


        ## tf2 transform setup
        # initialize listener
        self.tfBuf = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tfBuf)

        # initialize broadcaster
        self.mapToOdomBroad = tf2_ros.TransformBroadcaster()
        # Give the broadcaster time to connect, then send the initial transform.
        rospy.sleep(0.25)
        tfmsg = TransformStamped()
        tfmsg.header.stamp = rospy.Time.now()
        tfmsg.header.frame_id = 'map'
        tfmsg.child_frame_id = 'odom'
        tfmsg.transform = PlanarTransform.unity().toTransform()
        self.mapToOdomBroad.sendTransform(tfmsg)

        # Wait 30 seconds for a map at startup
        rospy.loginfo("Waiting for a map...")
        self.mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 30.0)

        # compute and save the wall proximity array
        print("Starting preprocessing.")
        self.walls_near = map_to_nearest_wall(self.mapmsg)

        # Create subscribers
        rospy.Subscriber('/odom', Odometry, self.cb_update_odom)
        rospy.Subscriber('/scan', LaserScan, self.cb_update_scan)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.cb_update_initialpose)


    # Callback for /odom
    def cb_update_odom(self, msg):
        self.odom = PlanarTransform.fromPose(msg.pose.pose)
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z

        # Publish transformed coordinates in map space (/pose)
        msg_pose = PoseStamped()
        msg_pose.header = msg.header
        msg_pose.header.frame_id = 'map'
        msg_pose.pose = (self.map_to_odom_TF * self.odom).toPose()
        self.pub_pose.publish(msg_pose)

    # Callback for /scan
    def cb_update_scan(self, msg):

        # convert from map frame to odom frame.
        # Assume we received a scan message (msg). Use TF to
        # look up the matching transform. Give it up to 100ms, in
        # case Linux has (temporarily) swapped out the odometry node.
        tfmsg = self.tfBuf.lookup_transform('odom', msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
        odom2laser = PlanarTransform.fromTransform(tfmsg.transform)

        ## compute a new map -> odom transform based on sensor data
        numRanges = len(msg.ranges)
        # create an empty matrix for J, A, and lambda (L)
        J = np.zeros((numRanges,3))
        A = np.zeros((numRanges,1))
        l = np.zeros(numRanges) # list to diagonalize into Lambda
        rowIdx = -1

        # iterate through all points in current scan view
        for i in range(0,numRanges,10):
            # Get the reported range
            rng = msg.ranges[i]
            # throw out any obvious garbage
            if (rng < msg.range_min) or (rng > msg.range_max):
                continue

            # increment the number of non-throwaway rows
            rowIdx += 1 # rowIdx = -1 at start

            # Compute the point (x,y) in LASER frame
            angle = msg.angle_min + msg.angle_increment*i
            laserx = rng * math.cos(angle)
            lasery = rng * math.sin(angle)

            # convert to ODOM frame (Checked in RVIZ: correct!)
            (odomx, odomy) = odom2laser.inParent(laserx, lasery)

            # convert to (latest) map frame for least squaring (Checked in RVIZ: correct!)
            (rx, ry) = self.map_to_odom_TF.inParent(odomx, odomy)

            # get nearest wall (Checked in  RVIZ: seems to be mostly correct but may have small offset?)
            (px, py) = self.walls_near[cartesiantopixel(self.mapmsg, rx, ry)]
            (px, py) = pixeltocartesian(self.mapmsg, px,py)

            d = math.sqrt((px-rx)*(px-rx) + (py-ry)*(py-ry))

            # create the least squares row
            J[rowIdx] = [(px - rx)/d, (py - ry)/d, (rx*py - ry*px)/d]
            # weight by distance from p
            l[rowIdx] = (1/(d))
            # set row A
            A[rowIdx, 0] = d

        # post-processing: remove all rows past rowIdx
        if (rowIdx != -1):
            J = J[0:rowIdx, :]
            A = A[0:rowIdx, :]
            l = l[0:rowIdx]
            L = np.diag(l)

            # Pseudoinverse!
            Delta = np.linalg.inv(J.T @ L @ J) @ J.T @ L @ A
            # print(rowIdx, J)
            # Delta = np.linalg.inv(J.T @ J) @ J.T @ A
            # check error
            # print(A - J@Delta)

            if self.vx > 0.5 or abs(self.wz) > 0.2:
                dx = Delta[0, 0] * UPDATE_FRACTION_MOVING_FAST
                dy = Delta[1, 0] * UPDATE_FRACTION_MOVING_FAST
                dt = Delta[2, 0] * UPDATE_FRACTION_MOVING_FAST
            elif self.vx > 0.1 or abs(self.wz) > 0.1:
                dx = Delta[0, 0] * UPDATE_FRACTION_MOVING_SLOW
                dy = Delta[1, 0] * UPDATE_FRACTION_MOVING_SLOW
                dt = Delta[2, 0] * UPDATE_FRACTION_MOVING_SLOW
            else:
                dx = Delta[0,0]*UPDATE_FRACTION_STOPPED
                dy = Delta[1,0]*UPDATE_FRACTION_STOPPED
                dt = Delta[2,0]*UPDATE_FRACTION_STOPPED

            # adjust our transform
            deltaAdj = PlanarTransform.basic(dx, dy, dt)
            # TODO: CHECK ORDER!!
            self.map_to_odom_TF = deltaAdj * self.map_to_odom_TF # THIS DOES THE UPDATING


        # publish map -> odom transform
        tfmsg = TransformStamped()
        tfmsg.header.stamp = msg.header.stamp
        tfmsg.header.frame_id = 'map'
        tfmsg.child_frame_id = 'odom'
        tfmsg.transform = self.map_to_odom_TF.toTransform()
        self.mapToOdomBroad.sendTransform(tfmsg)

    # Callback for /initialpose
    def cb_update_initialpose(self, msg):
        self.map_to_odom_TF = PlanarTransform.fromPose(msg.pose.pose)

        # query the odom to base transform
        odombasemsg = self.tfBuf.lookup_transform('odom', 'base', msg.header.stamp, rospy.Duration(0.1))
        odom2base = PlanarTransform.fromTransform(odombasemsg.transform)

        self.map_to_odom_TF *= odom2base.inv()

        # publish transform
        tfmsg = TransformStamped()
        tfmsg.header.stamp = rospy.Time.now()
        tfmsg.header.frame_id = msg.header.frame_id
        tfmsg.child_frame_id = 'odom'
        tfmsg.transform = self.map_to_odom_TF.toTransform()
        self.mapToOdomBroad.sendTransform(tfmsg)
#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('localization')

    # Instantiate the Driver object
    localizer = LocalizationObj()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Localization spinning...")
    rospy.spin()
    rospy.loginfo("Localization stopped.")
