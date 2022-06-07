#!/usr/bin/env python3
# A quick attempt to map using polar coordinates
# lots of code borrowed from wall_tracking

import math
import rospy
import tf2_ros
import numpy as np

# Variables: Bounding box for checking whether a line is the same
from utilities.map_utilities import PlanarPoint, Line, PolarLine, LineNearLine, PolarPoint, angleDiff
from utilities.PlanarTransform import PlanarTransform
from utilities.visualization import Visualization

from geometry_msgs.msg import Point, PolygonStamped, Twist
from visualization_msgs.msg import Marker

MAX_SLOPE_DIF = 0.1  # How much slopes can vary
MAX_NOISE_DIST = 0.1 # min acceptable line distance
MIN_LINE_LENGTH = 0.01
OCCLUSION_TIME_CONSTANT = 10.0
OCCLUSION_PERCENT_CAP = 0.8




class WallTracker():
    DELTA_SLOPE = 0.5 # How much slopes can vary
    DELTA_DISTANCE = 0.1
    DELTA_LENGTH = 0.5  # How much different the lengths can be
    MIN_LENGTH = 0.01

    def __init__(self):
        self.map = [] # Current best estimate of walls/obstacles
        self.viz = Visualization('/viz_map')
        self.vx = 0
        self.wz = 0

        ## initialize general tf2 listener (for laser -> odom)
        self.tf2_buf = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tf2_buf)

        rospy.Subscriber('/inst_walls', PolygonStamped, self.cb_measurement, queue_size = 1)
        rospy.Subscriber('/vel_cmd', Twist, self.cb_vel_cmd)

    def cb_vel_cmd(self, msg):
        # Grab the forward and spin (velocity) commands.
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    # Pulls in each measurement as a point
    def cb_measurement(self, msg):
        if (abs(self.wz) >= 0.2):
            return
        # key details
        header = msg.header
        ptList  = msg.polygon.points

        # pull out the current map to odom transform
        # tfmsg = self.tf2_buf.lookup_transform(msg.header.frame_id, 'map', msg.header.stamp, rospy.Duration(0.1))
        tfmsg = self.tf2_buf.lookup_transform('base', msg.header.frame_id, rospy.Time.now(), rospy.Duration(0.1))
        self.TF_baseToMap = PlanarTransform.fromTransform(tfmsg.transform)
        print("Count:",len(self.map))

        # create placeholder lists for inst. walls and map walls in polar coordinates
        polarMeasurement = []

        # read in the measurement
        measureLines = []
        for i in range(0, len(ptList), 2):
            p1 = PlanarPoint(ptList[i].x, ptList[i].y, header.stamp.to_sec())
            p2 = PlanarPoint(ptList[i+1].x, ptList[i+1].y, header.stamp.to_sec())
            confidence = ptList[i].z

            if p1.x == p2.x:
                continue

            line = Line(p1, p2, 1)

            if line.length < self.MIN_LENGTH:
                continue

            polarLine = line.toPolar(self.TF_baseToMap)

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
            polarMap.append(line.toPolar(self.TF_baseToMap))
        # polarMap.sort(key=lambda l : min(l.p1.t, l.p2.t))

        # make the map!
        overlappingWallsPolar, hiddenWallsPolar, badWallsPolar = self.updateMapPolarRays(polarMap, polarMeasurement)
        self.updateMapPolar(polarMeasurement, overlappingWallsPolar, hiddenWallsPolar, badWallsPolar)

        # Update the map with new measurements
        self.viz.drawWalls(self.map, msg.header.frame_id)


    def updateMapPolarRays(self, polarmap, measure):
        # simpler version of updateMapPolar that updates all visible walls.
        overlappingWalls = []
        hiddenWalls = []
        badWalls = []

        for wall in polarmap:
            # Pull out all overlapping lines
            lengthoccluded = 0
            near = 0

            for line in measure:
                if line.overlaps(wall):
                    if (not LineNearLine(MAX_NOISE_DIST, wall.line, line.line)):
                        if (wall.p1.r > line.rAtTheta(wall.p1.t) and wall.p2.r > line.rAtTheta(wall.p2.t)):
                            lengthoccluded += line.line.length * wall.p1.r/line.p1.r
                    else:
                        near += 1

            percentOccluded = lengthoccluded/wall.line.length
            occlusionThreshold = min((measure[0].line.p1.time - wall.line.p1.time)/OCCLUSION_TIME_CONSTANT,OCCLUSION_PERCENT_CAP)
            # print(percentOccluded > occlusionThreshold,near>0)
            if percentOccluded > occlusionThreshold:
                hiddenWalls.append(wall)
            elif (near > 0):
                # TOOD: fix
                overlappingWalls.append(wall)
            else:
                badWalls.append(wall)


        # self.map = newMap
        return overlappingWalls, hiddenWalls, badWalls

    def updateMapPolar(self, measure, overlap, hidden, bad):
        newwalls = []
        overlapwalls = []
        hiddenwalls = []
        badwalls =  []


        for m in measure:
            newwalls.append(m.line)
        for wall in overlap:
            # Merge with measurement
            overlapwalls.append(wall.line)

        # self.elideHiddenWalls(hiddenwalls)

        for wall in hidden:
            # Don't update
            hiddenwalls.append(wall.line)
        for wall in bad:
            # Reduce confidence
            badwalls.append(wall.line)

        # self.updateMap(newwalls, overlap, hiddenwalls, badwalls, measure)

        # TODO: TESTING
        newMap = []
        newMap.extend(newwalls)
        # newMap.extend(overlapwalls)
        newMap.extend(hiddenwalls)
        self.map = newMap

        # self.map = newMeasurement
        # return

    # Provide new confidence
    def confidenceFunction(self, line, wall):
        lineWeight = line.confidence + line.length/wall.length
        wallWeight = wall.confidence

        sumWeights = lineWeight + wallWeight

        newConfidence = 1 - (1/(sumWeights + 1))

        return newConfidence

    def updateMap(self, newwalls, overlap, hidden, bad, currentmap):
        newmap = []

        hasmerged = np.zeros(len(currentmap))

        for lineidx in range(len(overlap)):
            for wallidx in range(len(currentmap)):
                line = overlap[lineidx].line
                pLine = overlap[lineidx]
                wall = currentmap[wallidx].line
                pWall = overlap[lineidx]

                merged = False  # Has the new measurement been synthesized into the map yet?

                # Check that lines are collinear
                # print(line.slopeAngle, wall.slopeAngle)

                # print(abs(angleDiff(line.slopeAngle, wall.slopeAngle))%math.pi )
                # if abs(angleDiff(line.slopeAngle, wall.slopeAngle))%math.pi < math.pi/10.0:
                if abs(line.slope - wall.slope) < self.DELTA_SLOPE*line.slope:
                # if (pLine.overlaps(pWall) and LineNearLine(self.DELTA_DISTANCE,line, wall)):
                    # If lines are close enough together synthesize the two
                    newConfidence = self.confidenceFunction(line,wall)

                    # Check if extend line
                    if line.p1.dist(wall.p1) < self.DELTA_DISTANCE and line.p1.dist(wall.p2) > wall.length:
                        # extend from p1 on wall to p1 on line. Keep wall p2
                        # wall.redefine(line.p1, wall.p2, newConfidence)  # EDIT IN PLACE
                        newWall = Line(line.p1, wall.p2, newConfidence)
                        newmap.append(newWall)
                        hasmerged[wallidx] = True
                        continue

                    elif line.p1.dist(wall.p2) < self.DELTA_DISTANCE and line.p1.dist(wall.p1) > wall.length:
                        # extend from p2 on wall to p1 on line. Keep wall p1
                        # wall.redefine(wall.p1, line.p1, newConfidence)  # EDIT IN PLACE
                        newWall = Line(wall.p1, line.p1, newConfidence)
                        newmap.append(newWall)
                        hasmerged[wallidx] = True
                        continue

                    elif line.p2.dist(wall.p1) < self.DELTA_DISTANCE and line.p2.dist(wall.p2) > wall.length:
                        # extend from p1 on wall to p2 on line. Keep wall p2
                        # wall.redefine(line.p2, wall.p2, newConfidence)  # EDIT IN PLACE
                        newWall = Line(line.p2, wall.p2, newConfidence)
                        newmap.append(newWall)
                        hasmerged[wallidx] = True
                        continue
                    elif line.p2.dist(wall.p2) < self.DELTA_DISTANCE and line.p2.dist(wall.p1) > wall.length:
                        # extend from p2 on wall to p2 on line. Keep wall p1
                        # wall.redefine(wall.p1, line.p2, newConfidence)  # EDIT IN PLACE
                        newWall = Line(wall.p1, line.p2, newConfidence)
                        newmap.append(newWall)
                        hasmerged[wallidx] = True
                        continue

                    if LineNearLine(self.DELTA_DISTANCE, line, wall):
                        # Weighted average on the endpoints by confidence
                        # TODO: distance-based
                        l1w1 = line.p1.dist(wall.p1)
                        l1w2 = line.p1.dist(wall.p2)
                        l2w1 = line.p2.dist(wall.p1)
                        l2w2 = line.p2.dist(wall.p2)

                        mindist = min(l1w1, l1w2, l2w1, l2w2)

                        newWeight = newConfidence

                        if l1w1 == mindist:
                            # Merge l1w1 and l2w2
                            newP1 = self.mergePoints(line.p1, wall.p1, newWeight)
                            newP2 = self.mergePoints(line.p2, wall.p2, newWeight)
                        elif l1w2 == mindist:
                            # Merge l1w2 and l2w1
                            newP1 = self.mergePoints(line.p1, wall.p2, newWeight)
                            newP2 = self.mergePoints(line.p2, wall.p1, newWeight)
                        elif l2w1 == mindist:
                           # Merge l2w1 and l1w2
                            newP1 = self.mergePoints(line.p2, wall.p1, newWeight)
                            newP2 = self.mergePoints(line.p1, wall.p2, newWeight)
                        elif l2w2 == mindist:
                            # Merge l2w2 and l1w1
                            newP1 = self.mergePoints(line.p2, wall.p2, newWeight)
                            newP2 = self.mergePoints(line.p1, wall.p1, newWeight)

                        if newP1.x == newP2.x:
                            continue

                        # print(newConfidence)

                        # wall.redefine(newP1, newP2, newConfidence)  # EDIT IN PLACE
                        newWall = Line(newP1, newP2, newConfidence)
                        newmap.append(newWall)
                        hasmerged[wallidx] = True
                        merged = True
                        continue


            if not merged:
                if line.length > self.MIN_LENGTH:
                    newmap.append(line)

        # Now go through hidden, just add
        for wall in hidden:
            newmap.append(wall)

        # Go through new detected walls, add
        for wall in newwalls:
            newmap.append(wall)

        # Now go through bad walls, decrease confidence
        for wall in bad:
            wall.confidence = wall.confidence/4.0
            if wall.confidence > 0.25:
                newmap.append(wall) # Don't add if too low of confidence


        #
        # # Copy over walls to new map, decrease confidence
        # for i in range(len(walls)):
        #     if hasmerged[i] == False:
        #         # walls[i].confidence = walls[i].confidence/2.0
        #
        #         # if walls[i].confidence > 0.25:
        #         newmap.append(walls[i])


        self.map = newmap

        oldtime = rospy.Time.now().to_sec()

        # Convert to polar map
        polarmap = []
        for line in self.map:
            polarmap.append(line.toPolar(self.TF_baseToMap))

        polarmap.sort(key=lambda l : min(l.p1.t, l.p2.t))

        self.elideMapPolar(polarmap)


        print("update: ", rospy.Time.now().to_sec() - oldtime)

    def elideHiddenWalls(self, polarmap):
        # simpler version of updateMapPolar that updates all visible walls.
        numdel = 0
        for wall1idx in range(0, len(polarmap)-1):
            wall1idx = wall1idx - numdel
            wall2idx = wall1idx + 1

            wall1 = polarmap[wall1idx]
            wall2 = polarmap[wall2idx]

            # if wall1.line.length < self.MIN_LENGTH or wall2.line.length < self.MIN_LENGTH:
            #     continue

            # If different slopes, skip
            # if abs(angleDiff(wall1.slopeAngle, wall2.slopeAngle))%math.pi > math.pi / 10.0:
            if abs(wall1.slope - wall2.slope) > self.DELTA_SLOPE:
                continue

            # If far away, skip
            if not LineNearLine(self.DELTA_DISTANCE, wall1.line, wall2.line):
                continue

            # Otherwise, elide
            p1p1dist = wall1.line.p1.dist(wall2.line.p1)
            p1p2dist = wall1.line.p1.dist(wall2.line.p2)
            p2p1dist = wall1.line.p2.dist(wall2.line.p1)
            p2p2dist = wall1.line.p2.dist(wall2.line.p2)

            maxdist = max(p1p1dist, p1p2dist, p2p1dist, p2p2dist)

            if p1p1dist == maxdist:
                # Set w1p1 and w2p1 as new endpoints
                np1 = wall1.line.p1
                np2 = wall2.line.p1
            if p1p2dist == maxdist:
                # Set w1p1 and w2p2 as new endpoints
                np1 = wall1.line.p1
                np2 = wall2.line.p2
            if p2p1dist == maxdist:
                # Set w1p2 and w2p1 as new endpoints
                np1 = wall1.line.p2
                np2 = wall2.line.p1
            if p2p2dist == maxdist:
                # Set w1p2 and w2p2 as new endpoints
                np1 = wall1.line.p2
                np2 = wall2.line.p2

            newconfidence = (wall1.line.confidence + wall2.line.confidence) / 2
            newLine = Line(np1, np2, newconfidence)

            polarmap.pop(wall1idx)
            polarmap.insert(wall1idx, newLine.toPolar(self.TF_baseToMap))
            polarmap.pop(wall2idx)

            numdel += 1

    def mergePoints(self, p1, p2, alpha):
        newX = p1.x*(1-alpha) + p2.x*alpha
        newY = p1.y*(1-alpha) + p2.y*alpha
        return PlanarPoint(newX, newY, p1.time)




    def elideMapPolar(self, polarmap):
        walls = self.map

        # simpler version of updateMapPolar that updates all visible walls.
        numdel = 0
        for wall1idx in range(0, len(polarmap)-1):
            wall1idx = wall1idx - numdel
            wall2idx = wall1idx + 1

            wall1 = polarmap[wall1idx]
            wall2 = polarmap[wall2idx]

            # if wall1.line.length < self.MIN_LENGTH or wall2.line.length < self.MIN_LENGTH:
            #     continue

            # If different slopes, skip
            # if abs(angleDiff(wall1.line.slopeAngle, wall2.line.slopeAngle))%math.pi > math.pi / 10.0:

            if abs(wall1.line.slope - wall2.line.slope) > self.DELTA_SLOPE:
                continue

            # If far away, skip
            if not LineNearLine(self.DELTA_DISTANCE, wall1.line, wall2.line):
                continue

            # Otherwise, elide
            p1p1dist = wall1.line.p1.dist(wall2.line.p1)
            p1p2dist = wall1.line.p1.dist(wall2.line.p2)
            p2p1dist = wall1.line.p2.dist(wall2.line.p1)
            p2p2dist = wall1.line.p2.dist(wall2.line.p2)

            maxdist = max(p1p1dist, p1p2dist, p2p1dist, p2p2dist)

            if p1p1dist == maxdist:
                # Set w1p1 and w2p1 as new endpoints
                np1 = wall1.line.p1
                np2 = wall2.line.p1
            if p1p2dist == maxdist:
                # Set w1p1 and w2p2 as new endpoints
                np1 = wall1.line.p1
                np2 = wall2.line.p2
            if p2p1dist == maxdist:
                # Set w1p2 and w2p1 as new endpoints
                np1 = wall1.line.p2
                np2 = wall2.line.p1
            if p2p2dist == maxdist:
                # Set w1p2 and w2p2 as new endpoints
                np1 = wall1.line.p2
                np2 = wall2.line.p2

            newconfidence = (wall1.line.confidence + wall2.line.confidence) / 2
            newLine = Line(np1, np2, newconfidence)

            walls.pop(wall1idx)
            polarmap.pop(wall1idx)

            polarmap.insert(wall1idx, newLine.toPolar(self.TF_baseToMap))
            walls.insert(wall1idx, newLine)

            polarmap.pop(wall2idx)
            walls.pop(wall2idx)

            numdel += 1




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