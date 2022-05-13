#!/usr/bin/env python
#
#   PlanarTransform Class
#
#   Class to hold and process planar transforms, containing the state
#   (x,y,theta).  The angle is stored as the 3D quarternions would be,
#   i.e. as qz = sin(theta/2) and qw = cos(theta/2).  Methods are:
#
#     pt = PlanarTransform(px, py, qz, qw)
#     pt = PlanarTransform.unity()
#     pt = PlanarTransform.basic(x, y, theta)
#     pt = PlanarTransform.fromPose(pose)
#     pt = PlanarTransform.fromTransform(transform)
#
#     pose     = pt.toPose()            Convert to Pose
#     tranform = pt.toTransform()       Convert to Transform
#
#     pt2 = scale * pt1                 Scale
#     pt2 = pt1.inv()                   Invert
#     pt3 = pt1 * pt2                   Concatenate
#
#     (x,y) = pt.inParent(x,y)          Transform a point to parent frame
#
#     x     = pt.x()                    Extract x
#     y     = pt.y()                    Extract y
#     sin   = pt.sin()                  Extract sin(theta)
#     cos   = pt.cos()                  Extract cos(theta)
#     theta = pt.theta()                Compute theta
#
import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform


#
#  PlanarTransform Class Definition
#
class PlanarTransform:
    def __init__(self, px, py, qz, qw):
        self.px = px            # X coordinate
        self.py = py            # Y coordinate
        self.qz = qz            # sin(theta/2)
        self.qw = qw            # cos(theta/2)
        

    # Processing:
    def inParent(self, x, y):
        return (self.px + self.cos() * x - self.sin() * y,
                self.py + self.sin() * x + self.cos() * y)

    def __mul__(self, next):
        (x,y) = self.inParent(next.px, next.py)
        return PlanarTransform(x, y,
                               self.qz * next.qw + self.qw * next.qz,
                               self.qw * next.qw - self.qz * next.qz)

    def inv(self):
        return PlanarTransform(-self.cos() * self.px - self.sin() * self.py,
                                self.sin() * self.px - self.cos() * self.py,
                               -self.qz, self.qw)

    def __rmul__(self, scale):
        if self.qz == 0.0:
            return PlanarTransform(self.px*scale, self.py*scale, 0.0, 1.0)
        theta = self.theta()
        u = 0.5 * (self.qz + math.sin(theta * (scale-0.5))) / self.qz
        v = 0.5 * (self.qw - math.cos(theta * (scale-0.5))) / self.qz
        return PlanarTransform.basic(self.px*u - self.py*v,
                                     self.py*u + self.px*v,
                                     theta * scale)

    # Extraction:
    def x(self):
        return (self.px)

    def y(self):
        return (self.py)

    def sin(self):
        return (2.0 * self.qz * self.qw)

    def cos(self):
        return (self.qw**2 - self.qz**2)

    def theta(self):
        return math.atan2(self.sin(), self.cos())

    # Representation:
    def __repr__(self):
        return ("<px:%6.3f, py:%6.3f, qz:%6.3f, qw:%6.3f>"
                % (self.px, self.py, self.qz, self.qw))

    def __str__(self):
        return ("x %6.3fm, y %6.3fm, theta %7.3fdeg"
                % (self.px, self.py, self.theta() * 180.0/math.pi))

    # Convert to/from Pose and Transform:
    def toPose(self):
        return Pose(Point(self.px, self.py, 0.0),
                    Quaternion(0.0, 0.0, self.qz, self.qw))

    def toTransform(self):
        return Transform(Vector3(self.px, self.py, 0.0),
                         Quaternion(0.0, 0.0, self.qz, self.qw))

    @classmethod
    def unity(cls):
        return cls(0.0, 0.0, 0.0, 1.0)

    @classmethod
    def basic(cls, x, y, theta):
        return cls(x, y, math.sin(0.5*theta), math.cos(0.5*theta))

    @classmethod
    def fromPose(cls, pose):
        assert ((abs(pose.orientation.x) < 1e-9) and
                (abs(pose.orientation.y) < 1e-9)), "Pose not planar"
        return cls(pose.position.x, pose.position.y,
                   pose.orientation.z, pose.orientation.w)

    @classmethod
    def fromTransform(cls, transform):
        assert ((abs(transform.rotation.x) < 1e-9) and
                (abs(transform.rotation.y) < 1e-9)), "Transform not planar"
        return cls(transform.translation.x, transform.translation.y,
                   transform.rotation.z, transform.rotation.w)
