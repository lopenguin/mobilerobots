#!/usr/bin/env python3
# A quick attempt to map using polar coordinates
# lots of code borrowed from wall_tracking

import math
import rospy
import tf2_ros
import numpy as np

# Variables: Bounding box for checking whether a line is the same
from utilities.map_utilities import PlanarPoint, Line, PolarLine, LineNearLine, PolarPoint
from utilities.PlanarTransform import PlanarTransform
from utilities.visualization import Visualization

from geometry_msgs.msg import Point, PolygonStamped
from visualization_msgs.msg import Marker

MAX_SLOPE_DIF = 0.1  # How much slopes can vary
MAX_NOISE_DIST = 0.05 # min acceptable line distance
MIN_LINE_LENGTH = 0.05

class WallTracker():
    def __init__(self):
        self.map = [] # Current best estimate of walls/obstacles
        self.viz = Visualization('/viz_map')

        ## initialize general tf2 listener (for laser -> odom)
        self.tf2_buf = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tf2_buf)

        rospy.Subscriber('/inst_walls', PolygonStamped, self.cb_measurement)

    # Pulls in each measurement as a point
    def cb_measurement(self, msg):
        # key details
        header = msg.header
        ptList  = msg.polygon.points

        # pull out the current map to odom transform
        tfmsg = self.tf2_buf.lookup_transform(msg.header.frame_id, 'map', msg.header.stamp, rospy.Duration(0.1))
        self.TF_odomToMap = PlanarTransform.fromTransform(tfmsg.transform)
        print("Count:",len(self.map))

        # create placeholder lists for inst. walls and map walls in polar coordinates
        polarMeasurement = []
        polarMap = []

        # read in the measurement
        measureLines = []
        for i in range(0, len(ptList), 2):
            p1 = PlanarPoint(ptList[i].x, ptList[i].y, header.stamp.to_sec())
            p2 = PlanarPoint(ptList[i+1].x, ptList[i+1].y, header.stamp.to_sec())
            confidence = ptList[i].z

            line = Line(p1, p2, 1)
            polarLine = line.toPolar(self.TF_odomToMap)

            # TODO: include confidences
            polarMeasurement.append(polarLine)
            measureLines.append(line)

        # If map is empty, start from scratch using the measurement as the map
        if (len(self.map) == 0):
            self.map = measureLines
            self.viz.drawWalls(self.map, msg.header.frame_id)
            return

        # the measurement should already be sorted by the odom angle
        # sort the map by the odom angle
        polarMap = []
        for line in self.map:
            polarMap.append(line.toPolar(self.TF_odomToMap))
        # polarMap.sort(key=lambda l : min(l.p1.t, l.p2.t))

        # make the map!
        self.updateMapPolarRays(polarMap, polarMeasurement)

        # Update the map with new measurements
        self.viz.drawWalls(self.map, msg.header.frame_id)


    def updateMapPolarRays(self, polarmap, measure):
        # simpler version of updateMapPolar that updates all visible walls.
        newMap = []
        # initialize to 
        for polarline in measure:
            if (polarline.line.length > MIN_LINE_LENGTH):
                newMap.append(polarline.line)

        for wall in polarmap:
            # Pull out all overlapping lines
            overlaps = [line for line in measure if line.overlaps(wall)]

            # loop through overlapping lines; if any are in front, add the wall to map
            # behind = 0
            lengthoccluded = 0
            for line in overlaps:
                # Check if both endpoints of the measurement line are in front of the wall, if so keep the old wall
                if wall.radialDist(line.p1) < - MAX_NOISE_DIST and wall.radialDist(line.p2)  < - MAX_NOISE_DIST:
                    lengthoccluded += line.line.length

            if lengthoccluded/wall.line.length > min((line.line.p1.time - wall.line.p1.time)/10,.8):
                newMap.append(wall.line)

        self.map = newMap

#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('wall_tracker')

    # Instantiate the Odometry object
    ow = WallTracker()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Mapping...")
    rospy.spin()
    rospy.loginfo("Mapping stopped.")