#!/usr/bin/env python
#
#   wheelcontrol_skeleton.py
#
#   This is a skelenton for the implementation of the Wheel-Control node.  Beyond
#   the basic, it should
#     - stops if no commands are received after 0.25sec
#     - filter the commands to avoid spikes (high torque/current loads)
#     - filters the actual velocity
#     - adds feedback to perfectly achieve the velocity commands
#
#   Node:       /wheelcontrol
#   Publish:    /wheel_state            sensor_msgs/JointState
#               /wheel_desired          sensor_msgs/JointState
#   Subscribe:  /wheel_command          sensor_msgs/JointState
#
#   Other Inputs:   Encoder Channels (GPIO)
#   Other Outputs:  Motor Driver Commands (via I2C)
#
import math
import sys
import time
import rospy

from encoder import Encoder
from driver import Driver

from sensor_msgs.msg import JointState

## CONSTANTS
RATE = 100 # Hz
ENC_TO_RAD = 1.0/(16 * 45) * 2 * math.pi # rad / enc rev
VEL_TIME_CONST =  RATE / 5.0 # Hz
CMD_TIME_CONST = RATE / 50.0 # Hz
POS_TIME_CONST = RATE / 10.0 # Hz

PWM_SLOPE_L = 9.03114 # PWM / (rad/s)
START_INCPT_L = 60 # PWM val
PWM_SLOPE_R = 9.064  # PWM / (rad/s)
START_INCPT_R = 60 # PWM val

#
#   Command Callback Function
#
#   Save the command and the time received.
#
def callback_command(msg):
    global cmdvel
    global cmdtime

    # Note the current time (to timeout the command).
    now = rospy.Time.now()

    ## Save
    cmdvel = msg.velocity
    cmdtime = now

#
#   Timer Callback Function
#
def callback_timer(event):
    ## Setup
    global last_time
    global last_theta_L
    global last_thetadot_L
    global last_theta_R
    global last_thetadot_R

    global lastdesvel

    ## Note the current time to compute dt and populate the ROS messages.
    now = rospy.Time.now()
    dt = (now - last_time).to_sec()
    last_time = now

    ## Process the encoders, convert to wheel angles!
    # Get encoder readings
    theta_L = encoder.getLeft()*ENC_TO_RAD
    theta_R = encoder.getRight()*ENC_TO_RAD
    # Get derivatives
    thetadot_L = (theta_L - last_theta_L) / dt
    thetadot_R = (theta_R - last_theta_R) / dt
    # Filter derivatives
    thetadot_L = last_thetadot_L + VEL_TIME_CONST*dt*(thetadot_L - last_thetadot_L)
    thetadot_R = last_thetadot_R + VEL_TIME_CONST*dt*(thetadot_R - last_thetadot_R)
    # update last values
    last_theta_L = theta_L
    last_thetadot_L = thetadot_L
    last_theta_R = theta_R
    last_thetadot_R = thetadot_R

    ## Process the commands.
    # only run if command is recent
    despos = [0, 0]
    cmdPWM = [0, 0]
    desvel = [0 ,0]
    if ((now - cmdtime).to_sec() <= 1.0):
        # Filter cmd vel
        # desvel[0] = lastdesvel[0] + CMD_TIME_CONST*dt*(cmdvel[0] - lastdesvel[0])
        # desvel[1] = lastdesvel[1] + CMD_TIME_CONST*dt*(cmdvel[1] - lastdesvel[1])

        desvel = cmdvel

        # Euler integrate to get despos
        despos = [desvel[0]*dt, desvel[1]*dt]

        # # Add corrective factor for position offset
        # desvel[0] = desvel[0] + (theta_L - despos[0])/POS_TIME_CONST
        # desvel[1] = desvel[1] + (theta_R - despos[1])/POS_TIME_CONST

        # Generate motor commands (convert wheel speed to PWM)
        cmdPWM[0] = START_INCPT_L + PWM_SLOPE_L*desvel[0]
        cmdPWM[1] = -(START_INCPT_R + PWM_SLOPE_R*desvel[1])

        print(cmdPWM)

        # update last values
        lastdesvel = desvel


    # Command the wheels
    driver.left((cmdPWM[0]))
    driver.right((cmdPWM[1]))

    # Publish the actual wheel state
    msg = JointState()
    msg.header.stamp = now
    msg.name         = ['leftwheel', 'rightwheel']
    msg.position     = [theta_L, theta_R]
    msg.velocity     = [thetadot_L, thetadot_R]
    msg.effort       = cmdPWM
    pubact.publish(msg)

    # Publish the desired wheel state
    msg = JointState()
    msg.header.stamp = now
    msg.name         = ['leftwheel', 'rightwheel']
    msg.position     = despos
    msg.velocity     = desvel
    msg.effort       = cmdPWM
    pubdes.publish(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Global variables
    last_theta_L = 0
    last_thetadot_L = 0
    last_theta_R = 0
    last_thetadot_R = 0

    lastdesvel = [0, 0]
    cmdvel = [0, 0]
    cmdtime = 0

    # Initialize the ROS node.
    rospy.init_node('wheelcontrol')
    last_time = rospy.Time.now()
    cmdtime = last_time

    # Inititlize the low level.
    encoder = Encoder()
    driver  = Driver()

    # Create a publisher to send the wheel desired and actual (state).
    pubdes = rospy.Publisher('/wheel_desired', JointState, queue_size=10)
    pubact = rospy.Publisher('/wheel_state',   JointState, queue_size=10)

    # Create a subscriber to listen to wheel commands.
    sub = rospy.Subscriber("/wheel_command", JointState, callback_command)

    # Create the timer.
    duration = rospy.Duration(1.0/RATE);  # check!
    dt       = duration.to_sec()
    timer    = rospy.Timer(duration, callback_timer)


    # Spin while the callbacks are doing all the work.
    rospy.loginfo("Running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    timer.shutdown()

    # Clean up the low level.
    driver.shutdown()
    encoder.shutdown()
