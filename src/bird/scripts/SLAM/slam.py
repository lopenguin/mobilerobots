#!/usr/bin/env python3
#
#   slam.py
#
#   SLAM node.  This
#   (a) builds up a list of obstacles, allowing for moving obstacles/motions
#   (b) Fuses odometry and obstacle map for localization
#
#   Node:       /slam
#   Publish:    TF odom -> map          geometry_msgs/TransformStamped
#               /pose                   geometry_msgs/PoseStamped
#               /inst_walls             geometry_msgs/PolygonStamped
#               
#   Subscribe:  /scan_points            sensor_msgs/PointCloud2
#               TF laser -> odom        geometry_msgs/TransformStamped
#               /odom                   geometry_msgs/TransJointState
#

import rospy
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import numpy as np
import math

from utilities.map_utilities import MapPointsFromLaserScan, PlanarPoint, Line, LineNearLine
from utilities.PlanarTransform import PlanarTransform
from utilities.least_squares import LeastSquares
from utilities.visualization import Visualization

from geometry_msgs.msg import PoseStamped, PolygonStamped
from nav_msgs.msg      import Odometry

# constants
ERROR_THRESHOLD = 0.25
ENDPOINT_DISTANCE_THRESHOLD = 0.03
ADJACENT_DISTANCE_THRESHOLD = 0.04

class SLAM():
    def __init__(self):
        # world origin is initial robot position
        self.TF_mapToOdom = PlanarTransform.unity()

        self.lsqs = LeastSquares()
        self.laserProj = lg.LaserProjection()
        self.walls = []


        ## initialize publishers
        # Publishes to /pose
        self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)

        # Publishes odom -> map
        self.pub_mapToOdom = tf2_ros.TransformBroadcaster()
        rospy.sleep(0.5)
        self.pub_mapToOdom.sendTransform(self.TF_mapToOdom.toTransformStamped('map', 'odom', rospy.Time.now()))

        # visualization publisher
        self.viz = Visualization('/viz_instantaneous_lines')

        # publishes instantaneous walls
        self.pub_walls = rospy.Publisher('/inst_walls', PolygonStamped, queue_size=15)


        ## initialize general tf2 listener (for laser -> odom)
        self.tf2_buf = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tf2_buf)

        ## initialize subscribes (starts callbacks)
        rospy.Subscriber('/scan_points', PointCloud2, self.cb_scan, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.cb_odom)


    
    ## CALLBACKS
    # /odom callback (sensor_msgs/Odometry)
    def cb_odom(self, msg):
        odom = PlanarTransform.fromPose(msg.pose.pose)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        # Publish transformed coordinates in map space (/pose)
        msg_pose = PoseStamped()
        msg_pose.header = msg.header
        msg_pose.header.frame_id = 'map'
        msg_pose.pose = (self.TF_mapToOdom * odom).toPose()
        self.pub_pose.publish(msg_pose)


    # /scan callback (sensor_msgs/PointCloud2)
    def cb_scan(self, msg):
        # startTime = rospy.Time.now()

        # convert into (x, y) map frame points
        tfmsg = self.tf2_buf.lookup_transform('odom', msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
        TF_mapToBase = PlanarTransform.fromTransform(tfmsg.transform)

        pts = pc2.read_points(msg)
        scan = np.array(list(pts))
        for i in range(len(scan)):
            xscan = scan[i, 0]
            yscan = scan[i, 1]
            # transform!
            (xmap, ymap) = TF_mapToBase.inParent(xscan, yscan)
            scan[i, 0] = xmap
            scan[i, 1] = ymap
        # scan, minPts, maxPts = MapPointsFromLaserScan(msg, TF_mapToLaser) # list of points in map frame

        ## First: convert scan into list of possible walls (FINISHED, NOT DEBUGGED)
        # Algorithm:
        # 1. Add the first point in the scan to the current line.
        # 2. For each point in the scan:
        #   a) compute the least squares best fit line between the current line and the point.
        #   b) if the least squares best fit error is low, accept the point.
        #      otherwise, close out the current line and create a new line with the point.
        # 3. Once there are no more points, compute a least squares fit with
        #    the endpoints of the last line and the first line. Merge if error is low.

        instWalls = []  # instantaneous wall array
        currStart = 0
        currEnd = 0
        lastLineVals = (0.0, 0.0, 0.0, 0, 0)

        for i in range(scan.shape[0]):
            if (i == currStart):
                # want at least two points
                continue
            
            # check the distance from this point to the previous one
            d = np.sqrt((scan[i,0] - scan[i-1,0])**2 + (scan[i,1] - scan[i-1,1])**2)
            if (d > ADJACENT_DISTANCE_THRESHOLD):
                # make a wall with the current list points and move on with this one
                # d1 = np.sqrt((scan[currStart,0] - scan[currEnd,0])**2 + (scan[currStart,1] - scan[currEnd,1])**2)
                if (currStart >= currEnd):
                    currStart = i
                    continue
                l = buildLine(lastLineVals, currStart, currEnd, scan, instWalls)
                instWalls.append(l)
                currStart = i
                continue

            # create an array with the next n points (removing times)
            possibleLinePts = scan[currStart:(i+1), 0:2]

            # run least squares to get line of best fit
            if (currStart == i - 1):
                # send both points
                points = scan[currStart:(i+1), 0:2]
                intercept, slope, error = self.lsqs.reset(points)
            else:
                points = scan[i, 0:2]
                intercept, slope, error = self.lsqs.addPoint(points, possibleLinePts)

            # if (currStart >= currEnd):
            #     print('w')
            if (error > ERROR_THRESHOLD):
                # this line sucks... use up through the previous point
                # if (currStart >= currEnd):
                #     continue
                l = buildLine(lastLineVals, currStart, currEnd, scan, instWalls)
                instWalls.append(l)
                currStart = i
            else:
                # try adding the next point
                lastLineVals = (intercept, slope, error, currStart, i)
                currEnd = i

        self.walls = instWalls
        # # draw the lines when we are done
        self.viz.drawInstantaneousWalls(instWalls, 'odom')

        self.pub_walls.publish(self.wallsToPolygon(msg.header.stamp, 'odom'))
        
        self.pub_mapToOdom.sendTransform(self.TF_mapToOdom.toTransformStamped('map', 'odom', msg.header.stamp))

        # print((rospy.Time.now() - startTime).to_sec())

        # Update the map
        # self.startime = rospy.Time.now()
        # self.updateMap(instWalls)
        # # print("end time: ", (rospy.Time.now()-self.startime).to_sec())
        # # Draw the map
        # self.viz.drawInstantaneousWalls(self.walls, "odom")

    def wallsToPolygon(self, time, frame):
        points = []
        for wall in self.walls:
            pt1 = wall.p1.toPointMsg()
            pt2 = wall.p2.toPointMsg()

            pt1.z = wall.confidence
            pt2.z = wall.confidence
            points.append(pt1)
            points.append(pt2)
        
        msg = PolygonStamped()
        msg.header.stamp = time
        msg.header.frame_id = frame
        msg.polygon.points = points
        return msg



def buildLine(lastLineVals, currStart, currEnd, scan, instWalls):
    # this line sucks... use up through the previous point
    (b, m, e, startIdx, endIdx) = lastLineVals
    # startIdx = currStart
    # endIdx = currEnd

    # compute the line endpoints
    x = lambda x1, y1 : ((m*(y1 - b) + x1)/(1+m*m))
    startX = x(scan[startIdx,0], scan[startIdx, 1])
    startY = m*startX + b
    endX = x(scan[endIdx,0], scan[endIdx, 1])
    endY = m*endX + b
    endPt = PlanarPoint(endX, endY, scan[endIdx, 2])

    # combine endpoints if possible
    if len(instWalls) > 0:
        lastEndPt = instWalls[-1].p2
        dist = lastEndPt.distPt(startX, startY)
        if (dist > ENDPOINT_DISTANCE_THRESHOLD):
            # don't combine
            startPt = PlanarPoint(startX, startY, scan[startIdx, 2])
        else:
            startPt = lastEndPt
    else:
        startPt = PlanarPoint(startX, startY, scan[startIdx, 2])
    # startPt = PlanarPoint(startX, startY, scan[startIdx, 2])

    # add the wall and reset the start
    if e == 0.0:
        confidence = 1.0
    else:
        confidence = min(1.0-e, 1.0)# TODO: make confidence actually good

    return Line(startPt, endPt, confidence)

#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('slam')

    # Instantiate the Odometry object
    ow = SLAM()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("SLAMing...")
    rospy.spin()
    rospy.loginfo("SLAM stopped.")
