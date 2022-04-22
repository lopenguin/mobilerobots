#!/usr/bin/env python3
#
#   gyro.py
#
#   Create a Gyro object to read the gyroscope from the IMU.
#
import math
import smbus
import time


#
#   Gyro Object
#
#   This implements the gyro readings.
#
class Gyro:
    # I2C Definitions and Communication
    I2C_ADDR = 0x69
    WHO_AM_I = 0xEA

    BANKREG_ID = (0, 0x00)
    BANKREG_PWRMGMT1 = (0, 0x06)
    BANKREG_GYROCFG = (2, 0x01)
    BANKREG_GYROZ    = (0, 0x37)
    BANKREG_INTSTATUS1 = (0, 0x1A)

    REG_BANK_SEL = 0x7F

    # Set range: (range, bit divider)
    # RANGE = (250, 131)
    RANGE = (500, 65.5)
    # RANGE = (1000, 32.8)
    # RANGE = (2000, 16.4)

    def setBank(self, bank):
        self.i2cbus.write_byte_data(self.I2C_ADDR, self.REG_BANK_SEL, bank)

    def readReg(self, bankreg):
        self.setBank(bankreg[0])
        reg = bankreg[1]
        return self.i2cbus.read_byte_data(self.I2C_ADDR, reg)
    def writeReg(self, bankreg, byte):
        self.setBank(bankreg[0])
        reg = bankreg[1]
        self.i2cbus.write_byte_data(self.I2C_ADDR, reg, byte)

    def readRegList(self, bankreg, N):
        self.setBank(bankreg[0])
        reg = bankreg[1]
        return self.i2cbus.read_i2c_block_data(self.I2C_ADDR, reg, N)
    def writeRegList(self, bankreg, bytelist):
        self.setBank(bankreg[0])
        reg = bankreg[1]
        self.i2cbus.write_i2c_block_data(self.I2C_ADDR, reg, bytelist)


    # Initialize.
    def __init__(self, i2cbus, scale = math.radians(500.0)):
        # Save the I2C bus object.
        self.i2cbus = i2cbus

        # Confirm a connection to the IMU and gyro.
        if (self.readReg(self.BANKREG_ID) != self.WHO_AM_I):
            raise Exception("IMU not connected!")


        # power mgmt
        pwrmgmt1 = self.readReg(self.BANKREG_PWRMGMT1)
        self.writeReg(self.BANKREG_PWRMGMT1, pwrmgmt1 & ~(1<<6))

        # Set the gyroscope low-pass filter to 51.2Hz so we have to
        # sample at >=51.2Hz to avoid aliasing.
        self.writeReg(self.BANKREG_GYROCFG, 0x1B)

        # Wait 50ms to let the setup and filter change settle.
        time.sleep(0.05)

        # Set the gyro scale (default 500 deg/sec).  Feel free to change.
        self.setscale(scale)

        # Set the offset by calibration.
        self.offset = self.calibrate()

        # Assume the current reading is thus zero.
        self.reading = (0.0, False)

        # Report.
        print("Gyro enabled.")

    # Cleanup.
    def shutdown(self):
        # Nothing to do.
        pass


    # Set the Gyro scale (in rad/sec).
    def setscale(self, scale):
        # Compute the range (250, 500, 1000, or 2000 deg/sec).
        scalenum = int(math.ceil(math.log2(scale / math.radians(250.0))))
        scalenum = min(max(scalenum, 0), 3)

        # Determine and set the actual scale.
        self.scale = math.radians(250.0) * (2 ** scalenum)
        reg = self.readReg(self.BANKREG_GYROCFG)
        reg = reg & (0b11111001 | scalenum << 1)    # CHECK
        self.writeReg(self.BANKREG_GYROCFG, reg)

        # Let the change take effect before the next sample is read.
        time.sleep(0.01)

        # Report.
        print("Setting gyro scale to %.3f rad/sec (%.0f deg/sec)"
              % (self.scale, math.degrees(self.scale)))

    # Calibrate the Gyro Offset (assuming the IMU is not moving!).
    def calibrate(self, N = 200):
        # Report.
        print("Measuring the gyro offset - please put down/don't move")

        # Grab the samples.
        sum  = 0.0
        sum2 = 0.0
        for i in range(N):
            (speed, _) = self.readraw()
            sum  = sum  + speed
            sum2 = sum2 + speed**2
            time.sleep(0.01)
        avg = sum/N
        std = math.sqrt((sum2 - N*avg**2)/(N-1))

        # Report and check whether the std is above an acceptable
        # limit which would imply movement.
        stdlim = 0.1
        print("Gyro offset %.3f rad/sec (std %.3f <= %.3f limit)"
              % (avg, std, stdlim))
        if (std > stdlim):
            raise Exception("IMU was held or moving during gyro calibration")

        # Return the offset, being the average reading.
        return avg


    def readraw(self):
        # Grab the high (first) and low byte (second) in one read.
        bytes = self.readRegList(self.BANKREG_GYROZ, 2)

        # Convert into a signed 16bit number.
        value = (bytes[0] << 8) | (bytes[1] & 0xFF)  # combine bytes
        if (value > 32767):
            value -= 65536

        # Check for saturation.
        saturated = ((value > 32700) or (value < -32700))

        # Scale into rad/sec.
        omegaraw = math.radians(value / self.RANGE[1])

        # Return the speed and saturation flag.
        return (omegaraw, saturated)


    def read(self):
        # Place the code in a try statement, in case the read fails.
        try:
            # Take the reading.
            (omega, saturated) = self.readraw()

            # Subtract the offset and save the reading.
            self.reading = (omega - self.offset, saturated)

        except:
            # Do not update the reading.
            pass

        # Return the reading (speed and saturation flag).
        return self.reading


#
#   Main
#
if __name__ == "__main__":
    # Grab the I2C bus.
    i2cbus = smbus.SMBus(1)

    # Initialize the motor gyro.
    # gyro = Gyro(i2cbus, math.radians(200.0))
    gyro = Gyro(i2cbus)

    # Try reading.
    try:
        while 1:
            (omega, sat) = gyro.read()
            print("GyroZ = %7.3f rad/sec (sat = %d) " % (omega, sat))
            time.sleep(0.1)
    except:
        print("Breaking the loop...")

    # Cleanup (does nothing).
    gyro.shutdown()
