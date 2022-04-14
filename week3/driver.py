#!/usr/bin/env python3
#
#   driver.py
#
#   Create a Driver object to process the motor driver: Prepare and
#   send commands to the motor.
#
import qwiic_scmd
import time


#
#   Driver Object
#
#   This implements the left and right motor driver.  The channels are
#   I2c channels (not GPIO pins).  NOTE the channels and reversing may
#   be different!
#
class Driver:
    # Initialize.
    def __init__(self, chL = 0, chR = 1, reverseL = 0, reverseR = 0):
        # Pick/save the parameters.
        self.chL  = chL
        self.chR  = chR
        self.revL = reverseL
        self.revR = reverseR
    
        # Initialize a connection to the motor driver.
        self.driver = qwiic_scmd.QwiicScmd()
        if not self.driver.connected:
            raise Exception("Motor Driver not connected!")
        print("Motor driver connected.")

        # Begin and enable the motors (after setting commands to zero).
        self.driver.begin()
        self.driver.set_drive(0, 0, 0)
        self.driver.set_drive(1, 0, 0)
        self.driver.enable()
        print("Motors enabled.")


    # Cleanup.
    def shutdown(self):
        # Simply disable the motors.
        print("Disabling the motors...")
        self.driver.set_drive(0, 0, 0)
        self.driver.set_drive(1, 0, 0)
        self.driver.disable()


    # Set the motors:
    def left(self, pwm):
        pwm = min(max(pwm, -255.0), 255.0)
        self.driver.set_drive(self.chL, self.revL, int(pwm))

    def right(self, pwm):
        pwm = min(max(pwm, -255.0), 255.0)
        self.driver.set_drive(self.chR, self.revR, int(pwm))


#
#   Main
#
if __name__ == "__main__":
    # Initialize the motor driver.
    driver = Driver()

    # Try a sinple move.
    print('Test driving forward...');
    driver.left(110)
    driver.right(-110)
    time.sleep(1.0)

    print('Test spinning right (negative Z)...');
    driver.left(110)
    driver.right(110)
    time.sleep(1.0)

    # Cleanup (disabling the motors).
    driver.shutdown()
