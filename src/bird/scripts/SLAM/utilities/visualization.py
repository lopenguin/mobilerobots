#!/usr/bin/env python3
#
#   visualization.py
#

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from utilities.map_utilities import PlanarPoint


class Visualization:
    def __init__(self, name):
        # Create a publisher to send markers
        self.pub_marker = rospy.Publisher(name, Marker, queue_size=10)

    def drawLineFromPoints(self, pointslist, frame):

        # plot the marker
        markermsg = Marker()
        markermsg.header.frame_id = frame
        markermsg.header.stamp = rospy.Time.now()
        markermsg.ns = "waypoints"
        markermsg.id = 0

        markermsg.type = Marker.LINE_STRIP
        markermsg.action = Marker.ADD

        markermsg.color.r = 1.0
        markermsg.color.g = 1.0
        markermsg.color.a = 1.0

        markermsg.scale.x = 0.01
        markermsg.scale.y = 0.01

        markermsg.pose.orientation.w = 1.0

        markermsg.points = []
        for point in pointslist:
            markermsg.points.append(point.toPointMsg())

        self.pub_marker.publish(markermsg)

    def drawWalls(self, linelist, frame):
        # plot the marker
        markermsg = Marker()
        markermsg.header.frame_id = frame
        markermsg.header.stamp = rospy.Time.now()
        markermsg.ns = "waypoints"
        markermsg.id = 0

        markermsg.type = Marker.LINE_LIST
        markermsg.action = Marker.ADD

        markermsg.color.r = 1.0
        markermsg.color.g = 1.0
        markermsg.color.a = 1.0

        markermsg.scale.x = 0.025
        markermsg.scale.y = 0.025

        markermsg.pose.orientation.w = 1.0

        markermsg.points = []
        for line in linelist:
            markermsg.points.append(line.p1.toPointMsg())
            markermsg.points.append(line.p2.toPointMsg())

        self.pub_marker.publish(markermsg)


    def drawInstantaneousWalls(self, linelist, frame):
        # plot the marker
        markermsg = Marker()
        markermsg.header.frame_id = frame
        markermsg.header.stamp = rospy.Time.now()
        markermsg.ns = "waypoints"
        markermsg.id = 0

        markermsg.type = Marker.LINE_LIST
        markermsg.action = Marker.ADD

        markermsg.color.r = 1.0
        markermsg.color.g = 0.0
        markermsg.color.a = 1.0

        markermsg.scale.x = 0.01
        markermsg.scale.y = 0.01

        markermsg.pose.orientation.w = 1.0

        markermsg.points = []
        for line in linelist:
            markermsg.points.append(line.p1.toPointMsg())
            markermsg.points.append(line.p2.toPointMsg())

        self.pub_marker.publish(markermsg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('visualization')

    # Instantiate the Driver object
    vis = Visualization()
    rospy.sleep(0.25)

    p1 = PlanarPoint(0, 0, 0)
    p2 = PlanarPoint(1, 1, 0)
    p3 = PlanarPoint(1, 2, 0)
    vis.drawLine([p1,p2,p3])

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Visualization spinning...")
    rospy.spin()
    rospy.loginfo("Visualization stopped.")
