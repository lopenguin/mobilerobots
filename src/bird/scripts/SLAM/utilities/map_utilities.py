#!/usr/bin/env python3
#
#   map_utilities.py
#
#   Definitions for classes:
#       Point(x,y)
#       Line(p1,p2,confidence)

import math
import rospy

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, Quaternion

class PlanarPoint:
    def __init__(self, x, y, time):
        self.x = x
        self.y = y
        self.time = time

    def redefine(self, x, y):
        self.x = x
        self.y = y

    @classmethod
    def fromPolar(cls, r, theta, time):
        return cls(r*math.cos(theta), r*math.sin(theta), time) # TODO: Check this

    @classmethod
    def fromPointMsg(cls, pointmsg, time):
        return cls(pointmsg.x,pointmsg.y, time)

    def toPointMsg(self):
        return Point(self.x, self.y, 0)

    @classmethod
    def fromTuple(cls, xy, time):
        (x, y) = xy
        return cls(x,y, time)

    def toTuple(self):
        return (self.x, self.y)

    def dist(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def distPt(self, ox, oy):
        return math.sqrt((self.x - ox)**2 + (self.y - oy)**2)

    @classmethod
    def fromPose(cls, pose, time):
        assert ((abs(pose.orientation.x) < 1e-9) and
                (abs(pose.orientation.y) < 1e-9)), "Pose not planar"
        return cls(pose.position.x, pose.position.y, time)

    def toPose(self):
        return Pose(Point(self.x, self.y, 0.0),
                    Quaternion(0.0, 0.0, 0, 0))


    # convert point into polar coordinates centered around the robot's position
    def toPolar(self, TF_odomToMap):
        # convert coordinates into odom frame
        (xOdom, yOdom) = TF_odomToMap.inParent(self.x, self.y)
        r = math.sqrt(xOdom*xOdom + yOdom*yOdom)
        theta = math.atan2(yOdom, xOdom)
        return PolarPoint(r, theta, self.time)

    def __repr__(self):
        return "(" + str(round(self.x,4)) + ", " + str(round(self.y,4)) + ")"


class PolarSweep():
    def __init__(self, line, oLine = None):
        # save the line
        self.line = line
        self.originalLine = oLine

        t1 = math.atan2(line.p1.y, line.p1.x)
        t2 = math.atan2(line.p2.y, line.p2.x)

        # order the points
        if (t1 > t2):
            if (t1 - t2 <= math.pi):
                self.t1 = t2
                self.t2 = t1
            else:
                self.t1 = t1
                self.t2 = t2
        else:
            if (t2 - t1 <= math.pi):
                self.t1 = t1
                self.t2 = t2
            else:
                self.t1 = t2
                self.t2 = t1

    def shortenedLine(self, newT1, newT2):
        r1 = self.rAtTheta(newT1)
        r2 = self.rAtTheta(newT2)
        p1 = PlanarPoint(r1*math.cos(newT1), r1*math.sin(newT1), 0)
        p2 = PlanarPoint(r2*math.cos(newT2), r2*math.sin(newT2), 0)
        return Line(p1, p2, 1.0)

    def rAtTheta(self, theta):
        # r = b /(sin t - m cos t)
        return self.line.intercept / (math.sin(theta) - self.line.slope * math.cos(theta))

class PolarPoint():
    def __init__(self, r, theta, time):
        self.r = r
        self.t = theta
        self.time = time

    def toCartesian(self, TF_mapToOdom):
        # convert to carteisan
        xOdom = self.r*math.cos(self.t)
        yOdom = self.r*math.sin(self.t)
        # convert to map frame
        (x, y) = TF_mapToOdom.inParent(xOdom, yOdom)
        return PlanarPoint(x, y, self.time)

    def __repr__(self):
        return "(r=" + str(round(self.r,4)) + ", t=" + str(round(self.t,4)) + ")"

class Line:
    def __init__(self, p1, p2, confidence):
        self.p1 = p1
        self.p2 = p2
        self.confidence = confidence
        self.slope = (self.p2.y - self.p1.y)/(self.p2.x - self.p1.x)
        self.intercept = self.p1.y - self.slope*self.p1.x
        self.length = self.p1.dist(self.p2)
        self.slopeAngle = math.atan2(p2.y - p1.y, p2.x - p1.x)

    @classmethod
    def fromCoord(cls, x1, y1, x2, y2, confidence):
        p1 = PlanarPoint(x1,y1,0)
        p2 = PlanarPoint(x2,y2,0)
        return cls(p1, p2, confidence)

    def toTuple(self):
        return (self.p1.toTuple(), self.p2.toTuple())

    def vector(self):
        return np.array([self.p1.x - self.p2.x, self.p1.y - self.p2.y])

    def midpoint(self):
        xm = (self.p1.x + self.p2.x) / 2
        ym = (self.p1.y + self.p2.y) / 2
        return PlanarPoint(xm, ym, self.p1.time)

    def redefine(self, p1, p2, confidence):
        self.p1 = p1
        self.p2 = p2
        self.confidence = confidence
        self.slope = (self.p2.y - self.p1.y)/(self.p2.x - self.p1.x)
        self.intercept = self.p1.y - self.slope*self.p1.x
        self.length = self.p1.dist(self.p2)

    def toPolar(self, TF_odomToMap):
        return PolarLine(self.p1.toPolar(TF_odomToMap), self.p2.toPolar(TF_odomToMap), self)

    def distToPoint(self, p):
        d = abs((self.p2.x - self.p1.x)*(self.p1.y - p.y) - (self.p1.x - p.x)*(self.p2.y - self.p1.y)) / self.length
        return d

    def __repr__(self):
        return str(self.p1) + ", " + str(self.p2)

class PolarLine():
    def __init__(self, p1, p2, line):
        if (p1.t < p2.t):
            self.p1 = p1
            self.p2 = p2
        else:
            self.p1 = p2
            self.p2 = p1
        self.line = line

        if abs(p1.t - p2.t) > math.pi:
            self.wrapped = True
        else:
            self.wrapped = False

    def rAtTheta(self, theta):
        # r = b /(sin t - m cos t)
        return self.line.intercept / (math.sin(theta) - self.line.slope * math.cos(theta))

    # Point in front = negative, behind = positive
    def radialDist(self, pt):
        rAtPt = self.rAtTheta(pt.t)
        return pt.r - rAtPt

    # assumes theta between -pi and pi
    def inThetaRange(self, testTheta):
        if not self.wrapped:
            if (testTheta >= self.p1.t) and (testTheta <= self.p2.t):
                return True
            else:
                return False
        else:
            if (testTheta <= self.p1.t) or (testTheta >= self.p2.t):
                return True
            else:
                return False

    # BAD! Do not use
    def occludedBy(self, other):
        if (self.inThetaRange(other.p1.t) and self.inThetaRange(other.p2.t)):
            return self.p1.r < other.p1.r
        else:
            return False

    def overlaps(self, o):
        if (self.inThetaRange(o.p1.t) or self.inThetaRange(o.p2.t)) or \
           (o.inThetaRange(self.p1.t) or o.inThetaRange(self.p2.t)):
            return True
        else:
            return False

    def toCartesian(self, TF_mapToOdom):
        return Line(self.p1.toCartesian(TF_mapToOdom), self.p2.toCartesian(TF_mapToOdom), self.line.confidence)

    def __repr__(self):
        return str(self.p1) + ", " + str(self.p2)

"""
Generate a list of points in map space of the laser scan results

Params:
    laserscanmsg
    map_to_lidar_TF

Returns:
    np array of points (p1,p2,...) corresponding to laser scan points, but in map space
"""
MAX_RANGE_MARKER = 1
MIN_RANGE_MARKER = -1
def MapPointsFromLaserScan(laserscanmsg, TF_mapToLaser):
    # pull out angle information
    minAngle = laserscanmsg.angle_min
    maxAngle = laserscanmsg.angle_max
    angleInc = laserscanmsg.angle_increment

    angles_lidar = np.arange(minAngle, maxAngle + angleInc, angleInc)
    nAngles = angles_lidar.shape[0]

    # pull out times
    tStart = laserscanmsg.header.stamp.to_sec()
    tInc = laserscanmsg.time_increment

    times = np.arange(0, nAngles*tInc, tInc) + tStart

    # pull out ranges and create one big array
    ranges_lidar = np.array(laserscanmsg.ranges)
    lidarData = np.vstack([angles_lidar, ranges_lidar, times])
    lidarData = lidarData.T

    # remove out of range values
    minRange = laserscanmsg.range_min
    maxRange = laserscanmsg.range_max

    aboveMax = lidarData[ranges_lidar > maxRange]
    belowMin = lidarData[ranges_lidar < minRange]
    goodData = lidarData[(ranges_lidar > minRange) & (ranges_lidar < maxRange)]


    # convert good data to map frame
    x_lidar = goodData[:, 1] * np.cos(goodData[:, 0])
    y_lidar = goodData[:, 1] * np.sin(goodData[:, 0])

    [x, y] = TF_mapToLaser.inParent(x_lidar, y_lidar)
    x = x.reshape([x.shape[0],1])
    y = y.reshape([y.shape[0],1])

    # combine!
    goodTimes = goodData[:, 2].reshape(x.shape[0], 1)
    mapPts = np.hstack([x, y, goodTimes])

    return mapPts, belowMin, aboveMax


######################################################################
#
#   RELATIVE TO POINT
#
#   Proximity of Point to Point
#
def PointNearPoint(d, pA, pB):
    return ((pA[0] - pB[0]) ** 2 + (pA[1] - pB[1]) ** 2 <= d ** 2)

#
#   Proximity of Segment to Segment
#
def EndpointsNearSegmentInterior(d, sA, sB):

    # Precompute the relative vectors.
    ( vx,  vy) = (sB[1][0]-sB[0][0], sB[1][1]-sB[0][1])
    (r1x, r1y) = (sA[0][0]-sB[0][0], sA[0][1]-sB[0][1])
    (r2x, r2y) = (sA[1][0]-sB[0][0], sA[1][1]-sB[0][1])

    # Precompute the vector cross products (orthogonal distances).
    r1Xv = r1x*vy - r1y*vx
    r2Xv = r2x*vy - r2y*vx

    # If the endpoints are on opposite sides of the line, this test
    # becomes irrelevant.  Skip and report.
    if (r1Xv * r2Xv <= 0):
        return (True, False)

    # Finaly check the only closer endpoint sA[0]/sA[1] vs the segment
    # sB[0]-sB[1].  The farther endpoint can never be the shortest distance.
    vTv = vx**2 + vy**2
    if abs(r2Xv) < abs(r1Xv):
        r2Tv = r2x*vx + r2y*vy
        return (False, ((r2Tv    >=   0) and
                        (r2Tv    <= vTv) and
                        (r2Xv**2 <= vTv * d**2)))
    else:
        r1Tv = r1x*vx + r1y*vy
        return (False, ((r1Tv    >=   0) and
                        (r1Tv    <= vTv) and
                        (r1Xv**2 <= vTv * d**2)))

def SegmentNearSegment(d, sA, sB):
    # First check the endpoints-to-endpoints.
    if (PointNearPoint(d, sA[0], sB[0]) or
        PointNearPoint(d, sA[0], sB[1]) or
        PointNearPoint(d, sA[1], sB[0]) or
        PointNearPoint(d, sA[1], sB[1])):
        return True

    # Then check the endpoints to segment interiors.  This also
    # reports whether the endpoints are on opposites sides of the
    # segement.
    cross1, near = EndpointsNearSegmentInterior(d, sA, sB)
    if near:
        return True
    cross2, near = EndpointsNearSegmentInterior(d, sB, sA)
    if near:
        return True

    # If both crossed, the segments are intersecting.
    return (cross1 and cross2)


def SegmentCrossSegment(sA, sB):
    # Precompute the relative vectors.
    (ux, uy) = (sA[1][0]-sA[0][0], sA[1][1]-sA[0][1])
    (vx, vy) = (sB[1][0]-sB[0][0], sB[1][1]-sB[0][1])
    (rx, ry) = (sB[0][0]-sA[0][0], sB[0][1]-sA[0][1])

    # Precompute the vector cross products.
    uXv = ux*vy - uy*vx
    rXu = rx*uy - ry*ux
    rXv = rx*vy - ry*vx

    # Check the intersection.
    if (uXv > 0):
        return ((rXu > 0) and (rXu < uXv) and (rXv > 0) and (rXv < uXv))
    else:
        return ((rXu < 0) and (rXu > uXv) and (rXv < 0) and (rXv > uXv))

def LineCrossLine(lineA, lineB):
    sA = lineA.toTuple()
    sB = lineB.toTuple()
    return SegmentCrossSegment(sA, sB)

def LineNearLine(d, lineA, lineB):
    sA = lineA.toTuple()
    sB = lineB.toTuple()

    return SegmentNearSegment(d, sA, sB)

def angleDiff(t1, t2):
    return (t1-t2) - 2.0*math.pi * round(0.5*(t1-t2)/math.pi)