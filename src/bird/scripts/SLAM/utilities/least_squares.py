#!/usr/bin/env python3

import numpy as np

'''
INPUT: np array of points objects.
RETURNS: (intercept, slope, least squares error)
'''
def leastSquaresLine(arr):
    (npoints, dim) = np.shape(arr)
    assert (dim == 2), "Non planar point used for least squares regression"

    # compute the averages
    (xbar, ybar) = np.mean(arr, axis=0)     # column

    x2bar = np.mean(np.square(arr[:,0]))

    xybar = np.dot(arr[:,0], arr[:,1]) / npoints

    # compute slope
    slope = (xybar - xbar*ybar) / (x2bar - xbar*xbar)
    intercept = ybar - xbar*slope

    E = arr[:,1] - np.ones([arr.shape[0], 1])*intercept - arr[:,0]*slope
    error = np.linalg.norm(E)

    return intercept, slope, error

# A class to make least squares more efficient
class LeastSquares():
    def __init__(self):
        self.n = 0

        self.xsum = 0.0
        self.ysum = 0.0

        self.x2sum = 0.0
        self.xysum = 0.0

        self.points = np.array([])

    def reset(self, points):
        (npoints, dim) = np.shape(points)
        self.n = npoints
        assert (dim == 2), "Non planar point used for least squares regression"

        (self.xsum, self.ysum) = np.sum(points, axis=0)

        self.x2sum = np.sum(np.square(points[:,0]))

        self.xysum = np.dot(points[:,0], points[:,1])

        self.points = points

        return self.computeLeastSquares()

    def addPoint(self, point, pointsList):
        [dim] = np.shape(point)
        self.n += 1
        assert (dim == 2), "Non planar point used for least squares regression"

        # update sums
        self.xsum += point[0]
        self.ysum += point[1]

        self.x2sum += point[0] * point[0]
        self.xysum += point[0] * point[1]

        self.points = pointsList

        return self.computeLeastSquares()

    # computes based on current values
    def computeLeastSquares(self):
        assert self.n >= 2, "Must have two or more points for regression"

        # compute slope
        slope = (self.xysum*self.n - self.xsum*self.ysum) / (self.x2sum*self.n - self.xsum*self.xsum)
        intercept = self.ysum/self.n - self.xsum/self.n*slope

        E = self.points[:,1] - np.ones([self.points.shape[0], 1])*intercept - self.points[:,0]*slope
        error = np.linalg.norm(E)

        return intercept, slope, error