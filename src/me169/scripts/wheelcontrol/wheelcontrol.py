#!/usr/bin/env python3
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
import smbus

from encoder import Encoder
from driver_replacement import Driver
from gyro import Gyro

from sensor_msgs.msg import JointState

## CONSTANTS
LOW_LEVEL_RATE = 100 # Hz
ROS_SEND_RATE = 75 # Hz

ENC_TO_RAD = 1.0/(16 * 45) * 2 * math.pi # rad / enc rev
VEL_TIME_CONST =  LOW_LEVEL_RATE / 5.0 # Hz
CMD_TIME_CONST = LOW_LEVEL_RATE / 10.0 # Hz
POS_TIME_CONST = 20 / LOW_LEVEL_RATE # s

PWM_SLOPE_L = 9.03114 # PWM / (rad/s)
START_INCPT_L = 55 # PWM val
PWM_SLOPE_R = 9.064  # PWM / (rad/s)
START_INCPT_R = 55 # PWM val

#
#   Command Callback Function
#
#   Save the command (velocity) and time of receipt
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
    global last_radz

    global lastdesvel
    global lastdespos

    global desmsg
    global actmsg
    global newmsg


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


    ## Process the Gyro
    (radzdot, sat) = gyro.read()  # rad/s
    # NOTE: not working
    radz = last_radz
    if not sat:
        radz = dt*radzdot + last_radz   # rad
        # update last radz
        last_radz = radz


    ## Process encoder for theta dot
    enc_radzdot = encToGlobal(thetadot_L, thetadot_R)
    enc_radzdot = enc_radzdot[1]


    ## Process the commands.
    # only run if command is recent
    cmdPWM = [0, 0]
    desvel = [0 ,0]
    despos = lastdespos
    if ((now - cmdtime).to_sec() <= 1.0):
        # Filter cmd vel
        desvel[0] = lastdesvel[0] + CMD_TIME_CONST*dt*(cmdvel[0] - lastdesvel[0])
        desvel[1] = lastdesvel[1] + CMD_TIME_CONST*dt*(cmdvel[1] - lastdesvel[1])

        # Euler integrate to get despos
        despos = [desvel[0]*dt + lastdespos[0], desvel[1]*dt + lastdespos[1]]
        lastdespos = despos

        # Add corrective factor for position offset
        vcorrect = [0, 0]
        vcorrect[0] = (despos[0] - theta_L)/POS_TIME_CONST
        vcorrect[1] = (despos[1] - theta_R)/POS_TIME_CONST

        # Generate motor commands (convert wheel speed to PWM)
        cmdPWM[0] = START_INCPT_L*(cmdvel[0]!=0) + PWM_SLOPE_L*(desvel[0] + vcorrect[0])
        cmdPWM[1] = START_INCPT_R*(cmdvel[1]!=0) + PWM_SLOPE_R*(desvel[1] + vcorrect[1])

        cmdPWM[0] = math.floor(cmdPWM[0])
        cmdPWM[1] = math.floor(cmdPWM[1])

        # establish limits
        if (cmdPWM[0] > 255):
            cmdPWM[0] = 254
        if (cmdPWM[0] < -255):
            cmdPWM[0] = -254

        if (cmdPWM[1] > 255):
            cmdPWM[1] = 254
        if (cmdPWM[1] < -255):
            cmdPWM[1] = -254

        # update last values
        lastdesvel = desvel


    # Command the wheels
    driver.left(cmdPWM[0])
    driver.right(cmdPWM[1])


    newmsg = False

    # Publish the actual wheel state /wheel_state
    actmsg = JointState()
    actmsg.header.stamp = now
    actmsg.name         = ['leftwheel', 'rightwheel', 'gyro']
    actmsg.position     = [theta_L, theta_R, radz]
    actmsg.velocity     = [thetadot_L, thetadot_R, radzdot]
    actmsg.effort       = [cmdPWM[0], cmdPWM[1], 0.0]

    # Publish the desired wheel state /wheel_desired
    desmsg = JointState()
    desmsg.header.stamp = now
    desmsg.name         = ['leftwheel', 'rightwheel']
    desmsg.position     = [despos[0], despos[1]]
    desmsg.velocity     = [desvel[0], desvel[1]]
    desmsg.effort       = [cmdPWM[0], cmdPWM[1]]

    newmsg = True

def pub_timer(event):
    global newmsg
    if (newmsg):
        now = rospy.Time.now()

        # Publish the actual wheel state /wheel_state
        pubact.publish(actmsg)

        # Publish the desired wheel state /wheel_desired
        pubdes.publish(desmsg)

        newmsg = False


#
#   Encoder angular speed to global rad/s
#
def encToGlobal(enctdot_L, enctdot_R):
    ## Vehicle Constants
    R = 33.0 # mm
    d2 = 137.0 # mm (wheel-to-wheel spacing)

    vx = enctdot_L*R/2   + enctdot_R*R/2
    wz = -enctdot_L*R/d2 + enctdot_R*R/d2

    return (vx, wz)

#
#   Main Code
#
if __name__ == "__main__":
    # Grab the I2C bus.
    i2cbus = smbus.SMBus(1)

    # Global variables
    last_theta_L = 0
    last_thetadot_L = 0
    last_theta_R = 0
    last_thetadot_R = 0
    last_radz = 0

    lastdesvel = [0, 0]
    lastdespos = [0, 0]
    cmdvel = [0, 0]
    cmdtime = 0

    desmsg = JointState()
    actmsg = JointState()
    newmsg = False

    # Initialize the ROS node.
    rospy.init_node('wheelcontrol')
    last_time = rospy.Time.now()
    cmdtime = last_time

    # Inititlize the low level.
    encoder = Encoder()
    driver  = Driver(i2cbus)
    gyro    = Gyro(i2cbus)

    # Create a publisher to send the wheel desired and actual (state).
    pubdes = rospy.Publisher('/wheel_desired', JointState, queue_size=10)
    pubact = rospy.Publisher('/wheel_state',   JointState, queue_size=10)

    # Create a subscriber to listen to wheel commands.
    sub = rospy.Subscriber("/wheel_command", JointState, callback_command)

    # Create the low level timer.
    duration = rospy.Duration(1.0/LOW_LEVEL_RATE);  # check!
    dt       = duration.to_sec()
    timer    = rospy.Timer(duration, callback_timer)

    # Create the publishing timer
    pubduration = rospy.Duration(1.0/ROS_SEND_RATE)
    pubtimer = rospy.Timer(pubduration, pub_timer)


    # Spin while the callbacks are doing all the work.
    rospy.loginfo("Running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timers (if not already done).
    timer.shutdown()
    pubtimer.shutdown()

    # Clean up the low level.
    driver.shutdown()
    encoder.shutdown()
