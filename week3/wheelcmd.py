#!/usr/bin/env python3
#
#   wheelcmd.py  left_wheel_speed  right_wheel_speed
#
#   Continually (at 10Hz!) send the given (constant) wheel speed commands.
#
#   Node:       /wheelcmd
#   Publish:    /wheel_command          sensor_msgs/JointState
#   Subscribe:  -none-
#
import sys
import rospy

from sensor_msgs.msg import JointState


#
#   Generator Object
#
class Generator:
    # Initialize
    def __init__(self, leftspeed, rightspeed):
        # Initialize the (repeating) message data.
        self.msg = JointState()
        self.msg.name.append('leftwheel')
        self.msg.name.append('rightwheel')
        self.msg.velocity.append(leftspeed)
        self.msg.velocity.append(rightspeed)
    
        # Create a publisher to send wheel commands.
        self.pub = rospy.Publisher('/wheel_command', JointState, queue_size=10)
       
    # Send a message.
    def send(self):
        # Update the message with the current time and publish.
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)
       
    # Timer callback: Send a message.
    def timercallback(self, event):
        self.send()


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('wheelcmd')

    # Pull out the wheel command from the non-Ros parameters.
    argv = rospy.myargv(argv=sys.argv)
    if (len(argv) != 3):
        print("Using default: wheelcmd.py 1.0 1.0")
        leftspeed  = 1.0
        rightspeed = 1.0
    else:
        leftspeed  = float(argv[1])
        rightspeed = float(argv[2])
    
    # Create the generator object.
    generator = Generator(leftspeed, rightspeed)
    

    # Create the timer.
    duration = rospy.Duration(0.1)
    dt       = duration.to_sec()
    timer    = rospy.Timer(duration, generator.timercallback)

    # Spin while the callbacks are doing all the work.
    rospy.loginfo("Wheel commands: left %f rad/sec, right %f rad/sec" %
                  (leftspeed, rightspeed))
    rospy.loginfo("Sending every %f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    timer.shutdown()
