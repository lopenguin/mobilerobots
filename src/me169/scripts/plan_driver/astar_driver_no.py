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
from State import State, GridNode

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
        self.des_states = [State.Empty()]

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
        if (len(self.des_states) == 0):
            return

        des_state = self.des_states[0]
        des_x = des_state.x
        des_y = des_state.y
        des_t = des_state.t

        cur_x = self.cur_state.x
        cur_y = self.cur_state.y
        cur_t = self.cur_state.t

        d = des_state.dist(self.cur_state)

        if (d < 2*self.CLOSE_ENOUGH_DISTANCE) and len(self.des_states) > 1:
            self.des_states.pop()
            return

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
        # Clear the waypoint queue
        self.des_states.clear()

        # generate a series of waypoints that lead to the desired state.
        des_state = State.FromPose(msg.pose)

        # run A*, updates the waypoint queue
        if not self.astar(des_state):
            rospy.loginfo("Point not reachable...")
            return

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
        markermsg.scale.x = 0.2
        markermsg.scale.y = 0.2
        markermsg.points = [Point(0.0, 0.0, 0.0)]
        for pt in self.des_states:
            markermsg.points.append(Point(pt.x, pt.y, 0.0))

        self.pub_marker.publish(markermsg)

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



    ''' Planning functions '''
    # Computes a path avoiding obstacles to desired position. Returns true if path found.
    def astar(self, des_state):
        # copy the map
        astarmap = self.mapgrid.copy()

        # define markers in the map copy
        WALL = 100
        UNKNOWN_RANGE = [0, 99] # anything between 0 and WALL is unknown
        PROCESSED = -1  # this also elimates areas outside map

        start = GridNode.FromState(self.mapmsg, self.cur_state, 0, None)
        goal  = GridNode.FromState(self.mapmsg, des_state, 0, None)

        queue = []
        visited = []
        current = start
        while (current.x != goal.x and current.y != goal.y):
            # add neighbors of start, with special weight on neigbors in the forward direction
            check_increments = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
            # TODO: fix angles
            angles = [math.pi, 5*math.pi/4, 3*math.pi/2, 7*math.pi/4, 0, math.pi/4, math.pi/2, 3*math.pi/4]
            for c, a in zip(check_increments, angles):
                c = [current.x + c[0], current.y + c[1]]
                # check if still in the map
                if not (c[0] > astarmap.shape[1]) or \
                       (c[1] > astarmap.shape[1]) or \
                       (c[0] < astarmap.shape[0]) or \
                       (c[1] < astarmap.shape[0]):

                    # good to add spatially. Now check if the new position is a wall or visited
                    if (astarmap[c[0], c[1]] >= UNKNOWN_RANGE[0]) and \
                       (astarmap[c[0], c[1]] <= UNKNOWN_RANGE[1]):

                        # angleCost = angleDiff(current.t, a)/(2*math.pi) % 1 + 1
                        angleCost = math.sqrt(c[0]**2 + c[1]**2)
                        queue.append(GridNode(c[0], c[1], a, current.cost + angleCost, current))

            # all possible neighbors have been added
            # make sure the queue still has stuff
            if (len(queue) == 0):
                # no path!
                return False

            # close current node
            astarmap[current.x, current.y] = PROCESSED
            visited.append(current)

             # sort the queue by cost + predicted cost-to-go (Eulerian)
            queue.sort(key=lambda s: (s.cost + s.dist(goal)))

            # pull out lowest cost
            current = queue.pop(0)

        print("End")
        # the goal has been reached.
        n = current
        while (n.x != start.x and n.y != start.y):
            self.des_states.insert(0, State.FromNode(self.mapmsg, n))
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
