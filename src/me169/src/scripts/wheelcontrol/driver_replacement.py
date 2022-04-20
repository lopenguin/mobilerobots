#!/usr/bin/env python3
#
#   driver.py
#
#   Create a Driver object to process the motor driver: Prepare and
#   send commands to the motor.
#
import smbus
import time


#
#   Driver Object
#
#   This implements the left and right motor driver.  The channels
#   (zero or one) are on the driver board (not GPIO pins).  NOTE the
#   channels and reversing may be different!
#
class Driver:
    # I2C Definitions and Communication
    I2C_ADDR = 0x5D

    ID = 0xA9

    REG_ID     = 0x01
    REG_MOTORA = 0x20
    REG_MOTORB = 0x21
    REG_ENABLE = 0x70

    REG_MOTORX = {0:REG_MOTORA, 1:REG_MOTORB}

    def readReg(self, reg):
        return self.i2cbus.read_byte_data(self.I2C_ADDR, reg)
    def writeReg(self, reg, byte):
        self.i2cbus.write_byte_data(self.I2C_ADDR, reg, byte)


    # Initialize.
    def __init__(self, i2cbus, chL = 0, chR = 1, reverseL = 1, reverseR = 1):
        # Save the I2C bus object.
        self.i2cbus = i2cbus

        # Pick/save the parameters.
        self.chL  = chL
        self.chR  = chR
        self.revL = reverseL
        self.revR = reverseR
    
        # Confirm a connection to the motor driver.
        if (self.readReg(self.REG_ID) != self.ID):
            raise Exception("Motor Driver not connected!")
        print("Motor driver connected.")

        # Begin and enable the motors (after setting commands to zero).
        self.set_drive(0, 0, 0)
        self.set_drive(1, 0, 0)
        self.enable()
        print("Motors enabled.")

    # Cleanup.
    def shutdown(self):
        # Simply disable the motors.
        print("Disabling the motors...")
        self.set_drive(0, 0, 0)
        self.set_drive(1, 0, 0)
        self.disable()

    # Enable/Disable.
    def enable(self):
        self.writeReg(self.REG_ENABLE, 1)
    def disable(self):
        self.writeReg(self.REG_ENABLE, 0)


    # Set the motors:
    def set_drive(self, channel, reverse, value):
        # Process the value: Reverse if necessary (actually inverted),
        # scale to -127..127 (rounding to zero), and offset by 128.
        if not reverse:
            value = -value
        value = 128 + min(max(int(value/2), -127), 127)

        # Send to the driver.
        self.writeReg(self.REG_MOTORX[channel], value)

    def left(self, pwm):
        pwm = min(max(pwm, -255.0), 255.0)
        self.set_drive(self.chL, self.revL, int(pwm))

    def right(self, pwm):
        pwm = min(max(pwm, -255.0), 255.0)
        self.set_drive(self.chR, self.revR, int(pwm))


#
#   Main
#
if __name__ == "__main__":
    # Grab the I2C bus.
    i2cbus = smbus.SMBus(1)
    
    # Initialize the motor driver.
    driver = Driver(i2cbus)

    # Try the left and right motors individually.
    print('Test left only positive = forward...');
    driver.left(110)
    time.sleep(1.0)
    driver.left(0)

    print('Test right only positive = backward...');
    driver.right(110)
    time.sleep(1.0)
    driver.right(0)

    # Try a simple move.
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
