#!/usr/bin/env python3
#
#   plan_driver.py
#
#   Converts current position and desired position into velocity/twist
#
#   Node:       /plan_driver
#   Publish:    /vel_cmd          geometry_msgs/Twist
#   Subscribe:  /odom                   Odometry
#               /move_base_simple/goal  geometry_msgs/PoseStamped
#               /scan                   sensor_msgs/LaserScan
#
import math
import rospy
import numpy as np
from State import State, Node, cartesiantopixel, pixeltocartesian, growWalls

from geometry_msgs.msg  import Point, Quaternion, Twist
from geometry_msgs.msg  import PoseStamped
from sensor_msgs.msg    import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker


#
#   Simple Driver
#
class DriverObj():
    def __init__(self):
        ## define class variables
        # velocity/angle change to publish
        self.vx = 0
        self.wz = 0
        # current
        self.cur_state = State.Empty()
        # desired
        self.des_states = [State.Empty(), State(1.0, 1.0, 0.0)]

        self.stopped = False

        ## initalize node
        # Wait 30 seconds for a map at startup
        rospy.loginfo("Waiting for a map...")
        self.mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 30.0)

        # copy the map probabilities into a new array
        width = self.mapmsg.info.width # Map width [cells]
        height = self.mapmsg.info.height # Map height [cells]
        mapdata = self.mapmsg.data #Map data, in row-major order, starting with (0,0). Occupancy probabilities [0,100]. Unknown is -1.
        self.mapgrid = np.reshape(mapdata,[height,width]) # Map data in 2D array format (Top down)
        self.mapgrid = self.mapgrid.T # Transpose into [width, height]

        self.mapgrid = growWalls(self.mapgrid, 8)

        # Create a publisher to send velocity commands.
        self.pub_velcmd = rospy.Publisher('/vel_cmd', Twist, queue_size=10)

        # Create a publisher to send markers
        self.pub_marker = rospy.Publisher('/waypoints', Marker, queue_size=10)

        # Create a subscriber to listen to pose commands.
        rospy.Subscriber('/pose', PoseStamped, self.cb_update_current)

        # Create a subscriber to listen to pose commands from rviz
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_update_desired)

        # Create subscriber to listen to laser scan
        rospy.Subscriber('/scan', LaserScan, self.cb_update_scan)


    # constants for gotoPoint2
    DRIVE_TIME_CONSTANT = 5.0
    MAX_DRIVE_SPEED = 0.5
    TURNING_TIME_CONSTANT = 2.0
    MAX_TURN_SPEED = 0.5 # rad/s

    CLOSE_ENOUGH_DISTANCE = 0.05 # m
    CLOSE_ENOUGH_THETA = 0.5

    def gotoPoint2(self):
        nwaypoints = len(self.des_states)
        if (nwaypoints == 0):
            return
            
        if (nwaypoints > 1):
            # consider skipping to the next waypoint
            if (self.cur_state.dist(self.des_states[0]) < 0.5):
                self.des_states.pop(0)

        des_state = self.des_states[0]
        des_x = des_state.x
        des_y = des_state.y
        des_t = des_state.t

        cur_x = self.cur_state.x
        cur_y = self.cur_state.y
        cur_t = self.cur_state.t



        d = des_state.dist(self.cur_state)

        if d < self.CLOSE_ENOUGH_DISTANCE:
            self.vx = 0
            theta_diff_des = angleDiff(des_t, cur_t)
            self.wz = clamp(theta_diff_des / self.TURNING_TIME_CONSTANT, -self.MAX_TURN_SPEED, self.MAX_TURN_SPEED)

            if theta_diff_des < self.CLOSE_ENOUGH_THETA:
                self.wz = 0
                self.des_states.pop(0)
                print("Pop!")

        else:
            phi = math.atan2(des_y - cur_y, des_x - cur_x)
            theta_diff = angleDiff(phi, cur_t)
            self.vx = clamp(d*math.cos(theta_diff)/self.DRIVE_TIME_CONSTANT, 0.0 , self.MAX_DRIVE_SPEED)
            self.wz = clamp(theta_diff / self.TURNING_TIME_CONSTANT, -self.MAX_TURN_SPEED, self.MAX_TURN_SPEED)


    '''CALLBACK FUNCTIONS'''
    # Callback for /move_base_simple/goal
    def cb_update_desired(self, msg):
        assert (msg.header.frame_id == 'map'), "Message not in map frame"
        new_des = State.FromPose(msg.pose)
        # Clear the waypoint queue
        self.des_states.clear()
        # self.des_states.append(State.FromPose(msg.pose))

        # run A* to get to the desired.
        path = self.astar(new_des)
        if path:
            # add the desired goal to the very end
            self.des_states.append(new_des)
        

        # plot the marker
        markermsg = Marker()
        markermsg.header.frame_id = "map"
        markermsg.header.stamp = rospy.Time.now()
        markermsg.ns = "waypoints"
        markermsg.id = 0
        markermsg.type = Marker.POINTS
        markermsg.action = Marker.ADD
        markermsg.color.r = 1.0
        markermsg.color.g = 1.0
        markermsg.color.a = 1.0
        markermsg.scale.x = 0.05
        markermsg.scale.y = 0.05
        markermsg.points = []
        for pt in self.des_states:
            markermsg.points.append(Point(pt.x, pt.y, 0.0))

        self.pub_marker.publish(markermsg)

        # get moving!
        self.stopped = False


    # Callback for /odom
    def cb_update_current(self, msg):
        assert (msg.header.frame_id == 'map'), "Message not in map frame"
        # Update current x, y, theta, sin/cos
        self.cur_state = State.FromPose(msg.pose)
        # print(self.cur_state.x)
        # print(self.cur_state.y)
        # print(self.cur_state.t)

        if (not self.stopped):
            self.gotoPoint2()
        msg_velcmd = Twist()
        msg_velcmd.linear.x = self.vx
        msg_velcmd.linear.y = 0.0
        msg_velcmd.linear.z = 0.0
        msg_velcmd.angular.x = 0.0
        msg_velcmd.angular.y = 0.0
        msg_velcmd.angular.z = self.wz
        self.pub_velcmd.publish(msg_velcmd)

    ## Callback for /scan
    # constants
    ESTOP_OFFSET_MULT = 0.85 # 1/s(DANGER ZONE)
    ESTOP_OFFSET_MIN = 0.05 # m
    def cb_update_scan(self, msg):
        # stop if something is right in front
        range_min = msg.range_min
        range_max = msg.range_max

        if (self.vx < 0.0):
            # can't see anything!
            return
        
        offset = max(self.ESTOP_OFFSET_MULT*self.vx, self.ESTOP_OFFSET_MIN)

        for r in msg.ranges:
            # throw out anything outside of the range
            if (r < range_min) or (r > range_max):
                continue
            if (r < (range_min + offset)):
                self.stopped = True
                self.vx = 0
                # self.wz = 0
                break


    # A* function
    def astar(self, des):
        # copy the map
        astarmap = self.mapgrid.copy()
        [xmax, ymax] = astarmap.shape

        # define the key markers in the map
        WALL = 100
        PROCESSED = -1  # also eliminates regions outside the map
        ONDECK = -2

        # convert states to grid coorodinates
        (xs, ys) = cartesiantopixel(self.mapmsg, self.cur_state.x, self.cur_state.y)
        (xg, yg) = cartesiantopixel(self.mapmsg, des.x, des.y)

        # create the queue and visited lists
        queue = []
        visited = []

        # start at the start
        current = Node(xs, ys, None, 0.0)

        itnum = 0
        while (current.x != xg) or (current.y != yg):
            print(itnum)
            itnum+=1
            ## Add all neighbors
            incs_to_check = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
            for i in incs_to_check:
                # get the new coordinates
                c = [current.x + i[0], current.y + i[1]]
                # make sure we are still in the map
                if (c[0] >= 0) or \
                   (c[0] <= xmax) or \
                   (c[1] >= 0) or \
                   (c[1] <= ymax):

                    # we are good to add! Check if new node has a wall
                    val = astarmap[c[0], c[1]]
                    if (val != WALL) and (val != PROCESSED) and (val != ONDECK):
                        # add the new cost as Euclidian distance
                        newcost = current.cost + math.sqrt(i[0]*i[0] + i[1]*i[1])
                        queue.append(Node(c[0], c[1], current, newcost))
                        astarmap[c[0], c[1]] = ONDECK

            
            # all possible neighbors have been added.
            # if the queue is empty, there is no path
            if (len(queue) == 0):
                rospy.loginfo("No path found. Staying put.")
                return False

            # close out current node
            astarmap[current.x, current.y] = PROCESSED
            visited.append(current)

            # sort the queue by cost + predicted cost-to-go (Euler)
            queue.sort(key=lambda s: (s.cost + s.xydist(xg, yg)))

            # pull out lowest cost
            current = queue.pop(0)

        # Goal reached! Trace backwards to find path
        n = current
        while (n.x != xs or n.y != ys):
            (cx, cy) = pixeltocartesian(self.mapmsg, n.x, n.y)
            self.des_states.insert(0, State(cx, cy, 0.0))
            n = n.prev

        return True



# Compute the angle difference
def angleDiff(t1, t2):
    return (t1-t2) - 2.0*math.pi * round(0.5*(t1-t2)/math.pi)


# Clamp a variable between a maximum and minimum (Saturated)
def clamp(x, min_x, max_x):
    return max(min(max_x, x), min_x)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('astar_driver')

    # Instantiate the Driver object
    simpledriver = DriverObj()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Driver spinning...")
    rospy.spin()
    rospy.loginfo("Driver stopped.")
