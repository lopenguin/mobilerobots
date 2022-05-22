#!/usr/bin/env python3
#
#   maputilities.py
#

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose

import numpy as np

"""
Precompute the nearest wall for each grid on a map
Uses the "brush fire" algorithm for nearest wall

Input: OccupancyGrid Message
Returns: 2D array of tuples [(x,y),(x,y),(x,y)...] for the location of the nearest wall to each map grid point
"""

def map_to_nearest_wall(mapmsg):
    width = mapmsg.info.width # Map width [cells]
    height = mapmsg.info.height # Map height [cells]
    mapdata = mapmsg.data #Map data, in row-major order, starting with (0,0). Occupancy probabilities [0,100]. Unknown is -1.
    mapgrid = np.reshape(mapdata,[height,width]) # Map data in 2D array format (Top down)
    mapgrid = mapgrid.T # Transpose into [width, height]


    wallpts = np.zeros((0, 2), dtype=np.int)
    wallptmap = np.zeros((width, height, 2))
    # pthasbeenset = np.zeros((width, height), dtype='?') # Bitmap whether a grid cell has been set in wallptmap
    #
    #
    # # Find all coordinates that are exposed walls (not surrounded by other walls)
    # for v in range(height):
    #     for u in range(width):
    #         if mapgrid[u, v] > 50:
    #             # Also check the adjacent pixels in a 3x3 grid.
    #             adjacent = mapgrid[max(0, u - 1):min(width, u + 2), max(0, v - 1):min(height, v + 2)]
    #             if not np.all(adjacent > 50):
    #                 wallpts = np.vstack([wallpts, np.array([u, v])])
    #             else:
    #                 pthasbeenset[u, v] = True
    #
    # pointlist = wallpts.tolist()
    #
    # # Initialize all walls to be closest to themselves
    # for (wallx, wally) in pointlist:
    #     wallptmap[wallx, wally] = (wallx, wally)
    #     pthasbeenset[wallx, wally] = True
    #
    # # Brush fire until there are no cells left unset
    # while len(pointlist) is not 0:
    #     (u, v) = pointlist.pop(0)
    #     nearestwall = wallptmap[u, v]
    #
    #     for x in [max(0, u-1), u, min(width-1, u+1)]:
    #         for y in [max(0, v-1), v, min(height-1, v+1)]:
    #             if not pthasbeenset[x, y]:
    #                 wallptmap[x, y] = nearestwall
    #                 pthasbeenset[x, y] = True
    #                 pointlist.append((x, y))

        # if len(pointlist) is 0:
        #     pointlist = nextlist


    # Convert from pixel coordinates to cartesian coordinates
    # wallptmap_cart = np.zeros((width, height, 2), dtype=np.float)
    # res = mapmsg.info.resolution
    # origin = mapmsg.info.origin.position
    # ox = origin.x
    # oy = origin.y
    #
    # # TODO: Do this with matrix math
    # for v in range(height):
    #     for u in range(width):
    #         [px, py] = wallptmap[u, v]
    #         wallptmap_cart[u,v] = (ox + res*float(u), oy + res*float(v))

    for v in range(height):
        for u in range(width):
            if mapgrid[u, v] > 50:
                # Also check the adjacent pixels in a 3x3 grid.
                adjacent = mapgrid[max(0, u - 1):min(width, u + 2), max(0, v - 1):min(height, v + 2)]
                if not np.all(adjacent > 50):
                    wallpts = np.vstack([wallpts, np.array([u, v])])

    for v in range(height):
        for u in range(width):
            [px, py] = nearestwallpt(wallpts, u, v)
            wallptmap[u, v] = (px, py)

    return wallptmap






def nearestwallpt(wallpts, u, v):
    return wallpts[np.argmin(np.sum((np.array([u, v]) - wallpts) ** 2, axis=1))]


def pixeltocartesian(mapmsg,u,v):
    res = mapmsg.info.resolution
    origin = mapmsg.info.origin.position
    ox = origin.x
    oy = origin.y

    xg = ox + res*float(u)
    yg = oy + res*float(v)

    return (xg, yg)


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

if __name__  == "__main__":
    # Initialize the ROS node.
    rospy.init_node('maputilities')

    rospy.loginfo("Waiting for a map...")
    testmap = rospy.wait_for_message('/map', OccupancyGrid, 30.0)
    rospy.loginfo("Map received...")

    wallptmap = map_to_nearest_wall(testmap)

    # print(wallptmap)
    print("Finding the closest wall to: ")

    # print(wallptmap)
    (x,y) = (0,0)
    print(x,y)

    (u,v) = cartesiantopixel(testmap,x,y)
    (wu,wv) = wallptmap[u,v]
    print(pixeltocartesian(testmap,wu,wv))

