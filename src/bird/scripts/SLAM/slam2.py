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

from utilities.map_utilities import PlanarPoint, Line, LineNearLine, LineCrossLine, PolarSweep
from utilities.PlanarTransform import PlanarTransform
from utilities.least_squares import LeastSquares
from utilities.visualization import Visualization

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg      import Odometry

# constants
ERROR_THRESHOLD = 0.3
ENDPOINT_DISTANCE_THRESHOLD = 0.03
ADJACENT_DISTANCE_THRESHOLD = 0.04

LOCALIZE_LINE_DIST = 0.03
LOCALIZE_DELTA_THETA = 0.1

WALL_VEC_SLOPE_WEIGHT = 0.95
WALL_DIST_WEIGHTING = 0.5

LENGTH_CHANGE_PERCENT = 0.9
MIN_BLOCKED_LEN = 0.0

COMBINE_DISTANCE_THRESHOLD = 0.04
COMBINE_THETA_THRESHOLD = 0.04

class SLAM():
    def __init__(self):
        # world origin is initial robot position
        self.TF_mapToOdom = PlanarTransform.unity()
        # self.TF_mapToOdom = PlanarTransform(0.05, 0.05, 0.0, 1.0)
        self.vx = 0
        self.wz = 0

        # least squares setup
        self.lsqs = LeastSquares()
        self.laserProj = lg.LaserProjection()
        self.map = []

        ## initialize publishers
        # Publishes to /pose
        self.pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)

        # Publishes odom -> map
        self.pub_mapToOdom = tf2_ros.TransformBroadcaster()
        rospy.sleep(0.5)
        self.pub_mapToOdom.sendTransform(self.TF_mapToOdom.toTransformStamped('map', 'odom', rospy.Time.now()))

        # visualization publisher
        self.vizInst = Visualization('/viz_instantaneous_lines')
        self.vizPairs = Visualization('/viz_pairs')
        self.vizMap = Visualization('/viz_map')
        self.vizHidden = Visualization('/viz_map_hidden')


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

        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z

        # Publish transformed coordinates in map space (/pose)
        msg_pose = PoseStamped()
        msg_pose.header = msg.header
        msg_pose.header.frame_id = 'map'
        msg_pose.pose = (self.TF_mapToOdom * odom).toPose()
        self.pub_pose.publish(msg_pose)


    '''
    Main /scan callback (sensor_msgs/PointCloud2)
    Processes scan and adds to map
    '''
    def cb_scan(self, msg):
        if (self.vx > 0.01) or (self.wz > 0.01):
            return

        tfmsg = self.tf2_buf.lookup_transform('odom', msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))

        # read in the scan points (should be in base frame)
        pts = pc2.read_points(msg)
        scan = np.array(list(pts))
        for i in range(len(scan)):
            xscan = scan[i, 0]
            yscan = scan[i, 1]
            scan[i, 0] = xscan
            scan[i, 1] = yscan

        # convert to lines and draw in rviz
        scanLines_base = self.linesFromScan(scan)
        self.vizInst.drawInstantaneousWalls(scanLines_base, 'base')


        # initialize the map to the first measurement
        TF_odomToBase = PlanarTransform.fromTransform(tfmsg.transform)
        TF_mapToBase = self.TF_mapToOdom * TF_odomToBase
        if (len(self.map) == 0):
            # convert scan lines into map frame for storage
            self.map = [TF_mapToBase.lineInParent(l) for l in scanLines_base]
            self.vizMap.drawWalls(self.map, 'odom')
            # rospy.sleep(10)
            return
        scanLines_map = [TF_mapToBase.lineInParent(l) for l in scanLines_base]

        
        # LOCALIZATION: fit visible walls to the mesursurement
        closestWalls, otherWalls = getClosestWalls(self.map, scanLines_map)
        self.vizPairs.drawPairs(closestWalls, 'odom')
        # self.localize(closestWalls, scanLines_map)
        # # update based on the new transform
        # TF_mapToBase = self.TF_mapToOdom * TF_odomToBase
        # scanLines_map = [TF_mapToBase.lineInParent(l) for l in scanLines_base]

        # publish map -> odom transform
        tfmsg = TransformStamped()
        tfmsg.header.stamp = msg.header.stamp
        tfmsg.header.frame_id = 'map'
        tfmsg.child_frame_id = 'odom'
        tfmsg.transform = self.TF_mapToOdom.toTransform()
        self.pub_mapToOdom.sendTransform(tfmsg)


        # TODO: filter previous wall map to only include visible walls
        hiddenWalls, blockedWalls = checkOcclusion(scanLines_base, closestWalls, otherWalls, TF_mapToBase.inv())



        # MAPPING: adjust position of walls based on fit
        # TODO: IMPROVE SO PARTIALLY BLOCKED WALLS ARE NOT SHORTENED
        mapAdditions = self.updateMap(closestWalls, scanLines_map)
        visibleWalls = postProcess(mapAdditions)


        # save and publish the map!
        self.map = visibleWalls + hiddenWalls + blockedWalls
        self.vizMap.drawWalls(visibleWalls, 'odom')
        self.vizHidden.drawHiddenWalls(hiddenWalls + blockedWalls, 'odom')
        print("Count:", len(self.map))
        # print("Runtime:", round((rospy.Time.now() - startTime).to_sec(),3))


    '''
    Converts measurement points into a set of lines.
    '''
    def linesFromScan(self, scan):
        ## First: convert scan into list of possible walls
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

        # add the final line
        if (currEnd > currStart):
            l = buildLine(lastLineVals, currStart, currEnd, scan, instWalls)
            instWalls.append(l)

        return instWalls



    '''
    Fits wall map to current scan and adjusts transform if error small enough (TODO: not working)
    '''
    def localize(self, wallDict, scanMap):
        # create empty matricies
        numScanWalls = len(scanMap)
        A = np.zeros((2*numScanWalls, 3))
        B = np.zeros((2*numScanWalls, 1))
        l = np.zeros(2*numScanWalls) # list to diagonalize into Lambda

        rowIdx = 0

        for i in range(0, 2*numScanWalls, 2):
            sLine = scanMap[int(i/2)]
            nearWall = wallDict[sLine]
            if nearWall is None:
                continue

            # increment the number of non-throwaway rows
            rowIdx += 1 # rowIdx = -1 at start

            # wall unit vector
            w = nearWall.vector()
            w = w / np.linalg.norm(w)

            # scan unit vector
            v = sLine.vector()
            v = v / np.linalg.norm(v)

            # p is any point on wall
            p = [nearWall.p1.x, nearWall.p1.y]
            
            # r is the midpoint of the line
            rPt = sLine.midpoint()
            r = [rPt.x, rPt.y]

            # compute the distance between the lines
            d = nearWall.distToPoint(rPt)

            # create least-squares rows
            B[i,   0] = w[0]*v[0] + w[1]*v[1] - 1
            B[i+1, 0] = w[0]*(r[1] - p[1]) + w[1]*(p[0] - r[0])
            
            A[i]    = [   0,     0, v[0]*w[1] - v[1]*w[0]]
            A[i+1]  = [-w[1], w[0], r[0]*w[0] + r[1]*w[1]]

            # set the weighting
            l[i] = nearWall.length**2 * 1/d
            l[i+1] = l[i]

        # arrange lambda from l
        if (rowIdx <= 1):
            return
        A = A[0:2*rowIdx, :]
        B = B[0:2*rowIdx, :]
        l = l[0:2*rowIdx]
        L = np.diag(l)
        # regression!
        # Delta = -np.linalg.inv(A.T @ L @ A) @ A.T @ L @ B
        Delta = -np.linalg.inv(A.T @ A) @ A.T @ B
        dx = Delta[0,0]
        dy = Delta[1,0]
        dt = Delta[2,0]

        # adjust our transform
        deltaAdj = PlanarTransform.basic(dx, dy, dt)
        self.TF_mapToOdom = deltaAdj * self.TF_mapToOdom # THIS DOES THE UPDATING


    
    ''' 
    Updates the wall map based on new data (could use a little work)
    '''
    def updateMap(self, wallDict, scanMap):
        newMap = []
        # loop through all measurement lines and snap to nearest wall
        for sLine in scanMap:
            w = wallDict[sLine]
            
            if (w is None):
                # there was no wall nearby this scan line...
                # TODO: check if the wall is behind existing walls
                # for now just add
                newMap.append(sLine)

            else:
                # merge!
                # get the direction of wall and sline as vectors
                wallVec = w.vector()
                wallVec = np.append(wallVec, 0)
                sLineVec = sLine.vector()
                sLineVec = np.append(sLineVec, 0)
            
                # make sure the vectors aren't opposite and then average them
                rev = False
                if (np.dot(wallVec, sLineVec) < 0):
                    wallVec = -wallVec
                    rev = True
                avgVec = (wallVec*WALL_VEC_SLOPE_WEIGHT + sLineVec*(1 - WALL_VEC_SLOPE_WEIGHT))

                # compute the normal to the averaged vector pointing from sLine to wall
                normal = np.cross(np.cross(sLineVec, wallVec), avgVec)
                normal /= np.linalg.norm(normal)

                
                # get the endpoint distance vectors (from sLine to wall)
                if (rev):
                    # p1 matches with p2 and vice versa
                    d1 = np.array([sLine.p1.x - w.p2.x, sLine.p1.y - w.p2.y, 0])
                    d2 = np.array([sLine.p2.x - w.p1.x, sLine.p2.y - w.p1.y, 0])
                else:
                    d1 = np.array([sLine.p1.x - w.p1.x, sLine.p1.y - w.p1.y, 0])
                    d2 = np.array([sLine.p2.x - w.p2.x, sLine.p2.y - w.p2.y, 0])

                # project the distance vectors onto the averaged normal
                d1Proj = WALL_DIST_WEIGHTING * np.dot(normal, d1)*normal
                d2Proj = WALL_DIST_WEIGHTING * np.dot(normal, d2)*normal

                # add this to sLine's endpoints and convert into a PlanarPoint
                sP1 = np.array([sLine.p1.x, sLine.p1.y, 0])
                newP1 = sP1 + d1Proj
                newP1 = PlanarPoint(newP1[0], newP1[1], rospy.Time.now())

                sP2 = np.array([sLine.p2.x, sLine.p2.y, 0])
                newP2 = sP2 + d2Proj
                newP2 = PlanarPoint(newP2[0], newP2[1], rospy.Time.now())

                newLine = Line(newP1, newP2, 1.0)
                newMap.append(newLine)

        return newMap


'''
Processes the generated map to connect nearby lines
'''
def postProcess(mapAdditions):
    mergedCt = 0
    lines = []
    # step 1: create a list of all endpoints
    endpts = []
    for line in mapAdditions:
        endpts.append(line.p1)
        endpts.append(line.p2)

    # step 2: loop through each endpoint and compute the distance
    lineMerged = [False for i in range(len(mapAdditions))]
    for i in range(len(endpts)):
        # check if we've bypassed a line without merging it
        if (i % 2 == 0 and i != 0):
            # check the previous line
            idx = int(i/2) - 1
            if (not lineMerged[idx]):
                lines.append(mapAdditions[idx])

        targetPt = endpts[i]
        dists = [pt.dist(targetPt) for pt in endpts]
        # grab the minimum distance
        minPtDist = min(dists, key= lambda d : d if (d > 0) else 999)

        # check against distance threshold
        if (minPtDist < COMBINE_DISTANCE_THRESHOLD):
            idx = dists.index(minPtDist)
            line1Idx = int(i / 2)
            line2Idx = int(idx / 2)
            # make sure this is not just the line's other endpoint (unlikely)
            if (line1Idx != line2Idx):
                # combine the endpoints!
                p1 = endpts[i]
                p2 = endpts[idx]
                (newpx, newpy) = ((p1.x + p2.x)/2, (p1.y + p2.y)/2)

                p1.redefine(newpx, newpy)
                p2.redefine(newpx, newpy)

                # check if lines can be combined (dot product)
                # get unit vectors
                l1vec = mapAdditions[line1Idx].vector()
                l1vec /= np.linalg.norm(l1vec)
                l2vec = mapAdditions[line2Idx].vector()
                l2vec /= np.linalg.norm(l2vec)

                # take the dot product
                slopeDif = abs(np.dot(l1vec, l2vec)) - 1
                if abs(slopeDif) < COMBINE_THETA_THRESHOLD:
                    mergedCt += 1
                    # lines can be combined!
                    # pull out first and last point from two lines
                    if (p1 == mapAdditions[line1Idx].p1):
                        newP1 = mapAdditions[line1Idx].p2
                    else:
                        newP1 = mapAdditions[line1Idx].p1

                    if (p2 == mapAdditions[line2Idx].p1):
                        newP2 = mapAdditions[line2Idx].p2
                    else:
                        newP2 = mapAdditions[line2Idx].p1

                    # create the new line!
                    newLine = Line(newP1, newP2, 1.0)
                    mapAdditions[line1Idx] = newLine
                    mapAdditions[line2Idx] = newLine
                    lineMerged[line1Idx] = True

    
    # set the list of lines
    print("Merged:", mergedCt)
    return lines


'''
Check occlusion but simple
'''
def checkOcclusion(scanLines_base, closeWallsDict, otherWalls, TF_baseToMap):
    hiddenWalls = []
    for i in range(len(otherWalls)):
        w = otherWalls[i]
        wb = TF_baseToMap.lineInParent(w)
        # compute vector to midpoint
        mdpt = wb.midpoint()
        midLine = Line(PlanarPoint(0, 0, 0), mdpt, 1)
        for l in scanLines_base:
            if (LineNearLine(0.05,midLine, l)):
                hiddenWalls.append(w)
                break


    # check close walls for any partial occlusion
    # strategy: convert scan lines into polar coordinates
    #           convert close walls into polar coordinates
    # 1. Create sorted list of scan line endpoints (adjacent endpoints form line)
    # 2. Create sorted list of close walls
    # 3. Find walls that overlap (in theta) each scan line--ignoring the one "close" to the scan line
    # 4. Compute the blocked line and add to a list.

    # create sorted list of scan line endpoints
    polarScans = []
    for s in closeWallsDict.keys():
        # convert to base frame
        sb = TF_baseToMap.lineInParent(s)
        # convert endpoints to polar
        ps = PolarSweep(sb, s)
        polarScans.append(ps)
    polarScans.sort(key=lambda l : l.t2)

    blockedWalls = []
    for w in closeWallsDict.values():
        if (w == None):
            continue
        # convert to base frame
        wb = TF_baseToMap.lineInParent(w)
        # convert endpoints to polar
        pw = PolarSweep(wb, w)
        
        originalLineFound = False
        for s in polarScans:
            if not originalLineFound:
                if (pw.originalLine == closeWallsDict[s.originalLine]):
                    # don't compare original lines
                    originalLineFound = True
                    continue

            # look for overlap
            if (s.t2 - pw.t1 > - 0.2) and (s.t1 - pw.t2 < 0.2):
                # there is some partial blockage. Take the blocked segment and add it to a list
                blockedPart = pw.shortenedLine(max(s.t1, pw.t1), min(s.t2, pw.t2))
                if (blockedPart.length > MIN_BLOCKED_LEN):
                    # check if blocked part is truly blocked
                    mdpt = blockedPart.midpoint()
                    midLine = Line(PlanarPoint(0, 0, 0), mdpt, 1)
                    if (LineCrossLine(midLine, s.line)):
                        blockedPart_map = TF_baseToMap.inv().lineInParent(blockedPart)
                        blockedWalls.append(blockedPart_map)
                


    return hiddenWalls, blockedWalls
    # return hiddenWalls


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
    

''' 
creates a dictionary that maps a scan line to the closest wall
'''
def getClosestWalls(wallMap, scanMap):
    # empty dictionary of closest walls
    closestWalls = dict.fromkeys(scanMap)
    noConnectCt = 0

    wallHasNeighbor = [False for i in range(len(wallMap))]
    for sLine in scanMap:
        contenders = []
        for i in range(len(wallMap)):
            wall = wallMap[i]
            # generate a list of possible lines 
            if LineNearLine(LOCALIZE_LINE_DIST, sLine, wall):
                sVector = sLine.vector()
                sVector /= np.linalg.norm(sVector)
                wVector = wall.vector()
                wVector /= np.linalg.norm(wVector)
                slopeDif = abs(np.dot(sVector, wVector)) - 1
                if abs(slopeDif) < LOCALIZE_DELTA_THETA:
                    contenders.append(wall)
                wallHasNeighbor[i] = True

        if len(contenders) == 0:
            # wall has no nearby neighbors!
            noConnectCt += 1
            pass

        elif len(contenders) == 1:
            # easy choice
            closestWalls[sLine] = contenders[0]

        else:
            # pick the wall with the closest midpoint (TODO: make more robust)
            midpt = sLine.midpoint()
            minDist = 999.0
            choice = None
            for w in contenders:
                d = w.distToPoint(midpt)
                if d < minDist:
                    minDist = d
                    choice = w
            closestWalls[sLine] = choice

    # also save walls that don't match up to lines
    otherWalls = []
    for i in range(len(wallHasNeighbor)):
        b = wallHasNeighbor[i]
        if not b:
            otherWalls.append(wallMap[i])

    return closestWalls, otherWalls



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
