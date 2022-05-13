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