#!/usr/bin/env python3
#
# Defined x, y, theta class for points

import math

class State():
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t
        
    def dist(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    @classmethod
    def Empty(cls):
        return cls(0.0, 0.0, 0.0)

    @classmethod
    def FromPose(cls, pose):
        assert ((abs(pose.orientation.x) < 1e-9) and
                (abs(pose.orientation.y) < 1e-9)), "Pose not planar"
        return cls(pose.position.x, pose.position.y, 2*math.atan2(pose.orientation.z, pose.orientation.w))

    @classmethod
    def FromNode(cls, mapmsg, node):
        u = node.x
        v = node.y
        (x, y) = pixeltocartesian(mapmsg, u, v)
        return cls(x, y, node.t)


def growWalls(map, pxdist):
    map2 = map.copy()
    (xmax, ymax) = map.shape
    WALL = 100

    for i in range(xmax):
        for j in range(ymax):
            if (map[i, j] == WALL):
                for k in range(-pxdist, pxdist):
                    for l in range(-pxdist, pxdist):
                        if (i + k < xmax) and \
                           (i + k > 0) and \
                           (j + l < ymax) and \
                           (j + l > 0):
                            map2[i+k,j+l] = WALL

    return map2

# Class to hold a single node for A* planning. Uses GRID space
class Node():
    def __init__(self, x, y, prev, cost):
        self.x = x
        self.y = y
        self.prev = prev
        self.cost = cost

    def dist(self, o):
        return self.xydist(o.x, o.y)

    def xydist(self, ox, oy):
        return math.sqrt((self.x - ox)**2 + (self.y - oy)**2)


def cartesiantopixel(mapmsg,xm,ym):
    res = mapmsg.info.resolution
    width = mapmsg.info.width # Map width [cells]
    height = mapmsg.info.height # Map height [cells]
    origin = mapmsg.info.origin.position
    ox = origin.x
    oy = origin.y

    u = int(min(max(0, round((xm - ox)/res)), width-1))
    v = int(min(max(0, round((ym - oy)/res)), height-1))

    return (u, v)

def pixeltocartesian(mapmsg,u,v):
    res = mapmsg.info.resolution
    origin = mapmsg.info.origin.position
    ox = origin.x
    oy = origin.y

    xg = ox + res*float(u)
    yg = oy + res*float(v)

    return (xg, yg)