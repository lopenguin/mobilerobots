#!/usr/bin/env python3
#
#   teleop.py
#
#   Continually (at 10Hz!) send the a velocity command.
#
#   Node:       /teleop
#   Publish:    /vel_cmd          geometry_msgs/Twist
#   Subscribe:  -none-
#
import curses
import sys
import rospy

from geometry_msgs.msg import Twist


#
#   Terminal Input Loop
#
def loop(screen):
    # Make the getchar() non-blocking and otherwise initialize.
    curses.curs_set(0)
    curses.flushinp()
    screen.nodelay(True)
    screen.erase()
    screen.addstr(0, 0, "To move press and hold:    t y u")
    screen.addstr(1, 0, "(h = halt immediately)     g h j")
    screen.addstr(2, 0, "                           b n m")
    screen.addstr(3, 0, "Commands last 0.5sec")
    screen.addstr(4, 0, "Add SHIFT for high speed (2x)")
    screen.addstr(5, 0, "Add CTRL  for slow speed (0.25x)")
    screen.addstr(6, 0, "Hit q to quit")
    
    # Set up the map from keycodes (0..127) to velocities.
    nom  = [(0.0, 0.0)] * 0x20
    nom[ord('y') & 0x1F] = ( vnom,   0.0)   # y = forward
    nom[ord('n') & 0x1F] = (-vnom,   0.0)   # n = backward
    nom[ord('g') & 0x1F] = (  0.0,  wnom)   # g = left spin
    nom[ord('j') & 0x1F] = (  0.0, -wnom)   # j = right spin
    nom[ord('t') & 0x1F] = ( vnom,  wnom)   # t = foward + left
    nom[ord('u') & 0x1F] = ( vnom, -wnom)   # t = foward + right
    nom[ord('b') & 0x1F] = (-vnom,  wnom)   # b = backward + left
    nom[ord('m') & 0x1F] = (-vnom, -wnom)   # m = backward + right

    map  = []
    map += [(0.25*vx, 0.25*wz) for (vx, wz) in nom]   # Control   = slow
    map += [(0.0 *vx, 0.0 *wz) for (vx, wz) in nom]   # Numbers   = nothing
    map += [(2.0 *vx, 2.0 *wz) for (vx, wz) in nom]   # Uppercase = fast
    map += [(1.0 *vx, 1.0 *wz) for (vx, wz) in nom]   # Lowercase = nominal

    # Initialize the velocity and remaining active time.
    key = 0
    vel = (0.0, 0.0)
    Tactive = 0.0

    # Run the servo loop until shutdown.
    while not rospy.is_shutdown():

        # Reduce the active time and stop if necessary.
        Tactive -= dt
        if Tactive <= 0.0:
            vel = (0.0, 0.0)
            Tactive = 0.0

        # Process all pending keys.
        while True:
            keycode = screen.getch()
            if keycode == -1:
                break

            # Reset the active time.
            key     = keycode
            Tactive = 0.5

            # Change the velocity based on the key.
            if not ((keycode >= 0) and (keycode < 0x80)):
                vel = (0.0, 0.0)
            elif (keycode & 0x1F) == (ord('q') & 0x1F):
                rospy.signal_shutdown("quitting")       # q = quit
                return
            else:
                vel = map[keycode]

        # Report.
        if (key >= 0x20) and (key <= 0x80):
            s = "'" + str(chr(key)) + "'"
        else:
            s = "%d" % key
        screen.addstr(8, 0, "Last pressed 0x%02x = %s" % (key, s))
        screen.clrtoeol()
        screen.addstr(10, 0,
                      "Sending fwd = %6.3f m/s, spin = %6.3f rad/sec" % vel)

        # Update the message and publish.
        msg.linear.x  = vel[0]
        msg.angular.z = vel[1]
        pub.publish(msg)

        # Wait for the next turn.
        servo.sleep()


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('teleop')

    # Pull out the nominal forward/spin speeds from the non-ROS parameters.
    argv = rospy.myargv(argv=sys.argv)
    if (len(argv) > 3):
        print("Usage: teleop.py forward_speed spin_speed")
        print("GOOD DEFAULTS: teleop.py 0.2 1.0")
        sys.exit(0)
    elif (len(argv) < 3):
        print("Usage: teleop.py forward_speed spin_speed")
        print("Using default values: teleop.py 0.2 1.0")
        vnom = 0.2
        wnom = 1.0
    else:
        vnom = float(argv[1])
        wnom = float(argv[2])

    # Create a publisher to send twist commands.
    pub = rospy.Publisher('/vel_cmd', Twist, queue_size=10)

    # Initialize the (repeating) message data.
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    
    # Create a servo loop at 10Hz.
    servo = rospy.Rate(10)
    dt    = servo.sleep_dur.to_sec()

    # Report.
    rospy.loginfo("Teleop nominal fwd = %6.3f m/s, spin = %6.3f rad/sec"
                  % (vnom, wnom))
    rospy.loginfo("Teleop sending every %f sec..." % dt)

    # Run the terminal input loop.
    try:
        curses.wrapper(loop)
    except KeyboardInterrupt:
        pass

    # Report the exit.
    rospy.loginfo("Stopping transmissions.")
