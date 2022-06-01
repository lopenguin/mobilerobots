#!/usr/bin/env python3
# Receive:
#   Current best estimate for walls (A list of Line objects)
#   New measurement for walls (A list of Line objects)
#
# Produce:
#   Updated best estimate for walls (A list of Line objects, new confidences)

import math
import rospy
import numpy as np

# Variables: Bounding box for checking whether a line is the same
from utilities.map_utilities import PlanarPoint, Line, LineNearLine
from visualization_msgs.msg import Marker
from utilities.visualization import Visualization
from geometry_msgs.msg import Point, PolygonStamped



class WallTracker():
    DELTA_SLOPE = 1.0  # How much slopes can vary
    DELTA_DISTANCE = 0.1
    DELTA_LENGTH = 0.5  # How much different the lengths can be
    MIN_LENGTH = 0.1

    # visualization publisher
    def __init__(self):
        self.map = [] # Current best estimate of walls/obstacles
        self.viz = Visualization('/viz_map')

        rospy.Subscriber('/inst_walls', PolygonStamped, self.cb_measurement)

    # Pulls in each measurement as a point
    def cb_measurement(self, msg):
        # for now, just add each inst. wall to the list
        header = msg.header
        ptList = msg.polygon.points
        measurementWalls = []
        for i in range(0, len(ptList), 2):
            # line = ((ptList[i].x, ptList[i].y), (ptList[i+1].x, ptList[i+1].y))

            p1 = PlanarPoint(ptList[i].x, ptList[i].y, header.stamp.to_sec())
            p2 = PlanarPoint(ptList[i+1].x, ptList[i+1].y, header.stamp.to_sec())
            confidence = ptList[i].z

            # TODO: include confidences
            measurementWalls.append(Line(p1, p2, 1))

        # Update the map with new measurements
        self.updateMap(measurementWalls)
        self.viz.drawWalls(self.map, "odom")

    def elideMap(self):
        walls = self.map

        for wall1idx in range(0, len(walls)):
            for wall2idx in range(0,len(walls)):
                # If same wall, skip
                if wall1idx == wall2idx:
                    continue

                wall1 = walls[wall1idx]
                wall2 = walls[wall2idx]

                # If different slopes, skip
                if abs(wall1.slope - wall2.slope) > self.DELTA_SLOPE:
                    continue

                # If far away, skip
                if not LineNearLine(self.DELTA_DISTANCE, wall1, wall2):
                    continue

                # Otherwise, elide
                p1p1dist = wall1.p1.dist(wall2.p1)
                p1p2dist = wall1.p1.dist(wall2.p2)
                p2p1dist = wall1.p2.dist(wall2.p1)
                p2p2dist = wall1.p2.dist(wall2.p2)

                maxdist = max(p1p1dist, p1p2dist, p2p1dist, p2p2dist)

                if p1p1dist == maxdist:
                    # Set w1p1 and w2p1 as new endpoints
                    np1 = wall1.p1
                    np2 = wall2.p1
                if p1p2dist == maxdist:
                    # Set w1p1 and w2p2 as new endpoints
                    np1 = wall1.p1
                    np2 = wall2.p2
                if p2p1dist == maxdist:
                    # Set w1p2 and w2p1 as new endpoints
                    np1 = wall1.p2
                    np2 = wall2.p1
                if p2p2dist == maxdist:
                    # Set w1p2 and w2p2 as new endpoints
                    np1 = wall1.p2
                    np2 = wall2.p2

                newconfidence = (wall1.confidence + wall2.confidence) / 2
                newLine = Line(np1, np2, newconfidence)

                walls.pop(wall1idx)

                if wall1idx < wall2idx:
                    walls.pop(wall2idx-1)
                else:
                    walls.pop(wall2idx)

                walls.append(newLine)
                return True

        # Did not elide
        return False



    def updateMap(self, newMeasurement):
        walls = self.map
        addedwalls = []
        newmap = []

        hasmerged = np.zeros(len(walls))

        # Check if there are no walls. if no walls yet, build from scratch
        if len(walls) == 0:
            self.map = newMeasurement
            return

        for lineidx in range(len(newMeasurement)):
            for wallidx in range(len(walls)):
                line = newMeasurement[lineidx]
                wall = walls[wallidx]

                merged = False  # Has the new measurement been synthesized into the map yet?



                # Check that lines are collinear
                if abs(line.slope - wall.slope) < self.DELTA_SLOPE:
                    # If lines are close enough together synthesize the two
                    lineWeight = line.confidence + line.length
                    wallWeight = wall.confidence + wall.length

                    sumWeights = lineWeight + wallWeight
                    newConfidence = sumWeights
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
                        hasmerged[wallidx] = True

                        newmap.append(newWall)

                        continue
                    elif line.p2.dist(wall.p1) < self.DELTA_DISTANCE and line.p2.dist(wall.p2) > wall.length:
                        # extend from p1 on wall to p2 on line. Keep wall p2
                        # wall.redefine(line.p2, wall.p2, newConfidence)  # EDIT IN PLACE
                        newWall = Line(line.p2, wall.p2, newConfidence)
                        hasmerged[wallidx] = True

                        newmap.append(newWall)

                        continue
                    elif line.p2.dist(wall.p2) < self.DELTA_DISTANCE and line.p2.dist(wall.p1) > wall.length:
                        # extend from p2 on wall to p2 on line. Keep wall p1
                        # wall.redefine(wall.p1, line.p2, newConfidence)  # EDIT IN PLACE
                        newWall = Line(wall.p1, line.p2, newConfidence)
                        hasmerged[wallidx] = True

                        newmap.append(newWall)
                        continue

                    if LineNearLine(self.DELTA_DISTANCE, line, wall):

                        # Weighted average on the endpoints by confidence
                        # TODO: distance-based
                        l1w1 = line.p1.dist(wall.p1)
                        l1w2 = line.p1.dist(wall.p2)
                        l2w1 = line.p2.dist(wall.p1)
                        l2w2 = line.p2.dist(wall.p2)

                        mindist = min(l1w1, l1w2, l2w1, l2w2)

                        lineWeight = line.confidence + line.length
                        wallWeight = 2*wall.confidence + 2*wall.length
                        sumWeights = lineWeight + wallWeight

                        newWeight = lineWeight/sumWeights

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

                        newConfidence = sumWeights # TODO: better way?
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
            #TODO: else?

        # Copy over walls to new map
        for i in range(len(walls)):
            if hasmerged[i] == False:
                newmap.append(walls[i])

        self.map = newmap

        while (self.elideMap()):
            continue

        print(len(self.map))

    def mergePoints(self, p1, p2, alpha):
        newX = p1.x*(1-alpha) + p2.x*alpha
        newY = p1.y*(1-alpha) + p2.y*alpha
        return PlanarPoint(newX, newY, p1.time)


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

