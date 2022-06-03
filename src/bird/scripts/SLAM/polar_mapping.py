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
        
        if (len(self.map) == 0):
            self.map = measureLines
            self.viz.drawWalls(self.map, msg.header.frame_id)
            return

        # the measurement should already be sorted by the odom angle
        # sort the map by the odom angle
        polarMap = []
        for line in self.map:
            polarMap.append(line.toPolar(self.TF_odomToMap))
        polarMap.sort(key=lambda l : min(l.p1.t, l.p2.t))

        # prune the map: remove any walls that are completely occluded by other walls
        # polarMap = prune(polarMap)

        # make the map!
        self.updateMapPolarRays(polarMap, polarMeasurement)

        # Update the map with new measurements
        self.viz.drawWalls(self.map, msg.header.frame_id)


    def updateMapPolarRays(self, map, measure):
        # simpler version of updateMapPolar that updates all visible walls.
        newMap = []
        # initialize to 
        for line in measure:
            if (line.line.length > MIN_LINE_LENGTH):
                newMap.append(line.line)

        for wall in map:
            # Pull out all overlapping lines
            overlaps = [line for line in measure if line.overlaps(wall)]

            # loop through overlapping lines; if any are in front, add the wall to map
            behind = 0
            for line in overlaps:
                ## Compute distance between lines using measurement endpoints
                # exclude endpoints that don't overlap
                minDist = 999
                if (wall.inThetaRange(line.p1.t)):
                    minDist = wall.radialDist(line.p1)
                if (wall.inThetaRange(line.p2.t)):
                    d = wall.radialDist(line.p2)
                    if (abs(d) < abs(minDist)):
                        minDist = d
                if (line.inThetaRange(wall.p1.t)):
                    d = line.radialDist(wall.p1)
                    if (abs(d) < abs(minDist)):
                        minDist = d
                if (line.inThetaRange(wall.p2.t)):
                    d = line.radialDist(wall.p2)
                    if (abs(d) < abs(minDist)):
                        minDist = d

                if (minDist > MAX_NOISE_DIST) and (line.line.length > MIN_LINE_LENGTH):
                    behind += 1
            
            if (behind == len(overlaps)):
                newMap.append(wall.line)

        self.map = newMap


    def updateMapPolar(self, map, measure):
        # loop through the measurement and use polar coordinates
        for wall in map:
            # Pull out all overlapping lines
            overlaps = [line for line in measure if line.overlaps(wall)]

            # loop through the overlapping lines; if any are nearby, merge.
            for line in overlaps:
                ## Compute distance between lines using measurement endpoints
                # exclude endpoints that don't overlap
                minDist = 999
                if (wall.inThetaRange(line.p1.t)):
                    minDist = wall.radialDist(line.p1)
                if (wall.inThetaRange(line.p2.t)):
                    minDist = min(minDist, wall.radialDist(line.p2))
                if (line.inThetaRange(wall.p1.t)):
                    minDist = min(minDist, line.radialDist(wall.p1))
                if (line.inThetaRange(wall.p2.t)):
                    minDist = min(minDist, line.radialDist(wall.p2))

                ## Compare lines: slope
                if abs(line.line.slope - wall.line.slope) < MAX_SLOPE_DIF:
                    if (minDist < MAX_NOISE_DIST):
                        # combine the lines!
                        self.combineLines(wall, line)
                        pass
                    else:
                        # Lines are fairly far apart and parallel
                        # check which occludes which and update map
                        self.updateOcclusion(line, wall)
                else:
                    # lines are NOT parallel.
                    # if lines nearby, map has moved?
                    # TODO!!! (this case is not critical)
                    pass

        self.postProcess()


    # combines two lines and updates globalLine in self.map
    def combineLines(self, globalLine, newLine):
        # TODO: something more complicated than just averaging endpoints
        pt1 = PolarPoint((globalLine.p1.r + newLine.p1.r)/2, \
            (globalLine.p1.t + newLine.p1.t)/2, rospy.Time.now())
        pt2 = PolarPoint((globalLine.p2.r + newLine.p2.r)/2, \
            (globalLine.p2.t + newLine.p2.t)/2, rospy.Time.now())
            
        globalLine.line.redefine(pt1.toCartesian(self.TF_odomToMap.inv()), \
                            pt2.toCartesian(self.TF_odomToMap.inv()), \
                            globalLine.line.confidence)
        

    # checks which line occludes the other and updates accordingly
    def updateOcclusion(self, globalLine, newLine):
        # if (globalLine.occludedBy(newLine)):
        #     # newLine is completely in front of the global line...
        #     # add the new line to the map
        #     self.map.append()
        # TODO
        pass
        

    # checks current map for segments that have nearby endpoints
    def postProcess(self):
        pass


# removes lines that are occluded by other lines
def prune(polarMap):
    newMapList = [polarMap[0]]
    checkLine = polarMap[0]
    for i in range(1, len(polarMap)):
        l = polarMap[i]
        if not l.occludedBy(checkLine):
            newMapList.append(l)
            checkLine = l

    # cover the zero case
    for i in range(0, len(polarMap) - 1):
        l = polarMap[i]
        if l.occludedBy(checkLine):
            # remove the line
            newMapList.pop(0)
        else:
            # stop if no longer occluded
            break
    return newMapList


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