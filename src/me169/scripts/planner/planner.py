#!/usr/bin/env python3
#
#   planner.py
#
#   Planner node.  This
#   (a) Sends point-to-point commands to the driver node
#
#   Node:       /planner
#   Publish:    /move_base_simple/goal geometry_msgs/PoseStamped
#   Subscribe:
#               /scan
#