#!/usr/bin/env python
#
#   Drive the Robot
#
import sys
import time
import qwiic_scmd


# Define the constants.
R_MTR = 0
L_MTR = 1
FWD = 0
BWD = 1


#
#   Drive Function
#
def driveBot(driver):
    # Set speed to zero to enable.
    print('Enabling');
    driver.set_drive(R_MTR, FWD, 0)
    driver.set_drive(L_MTR, FWD, 0)
    driver.enable()

    # Wait.
    time.sleep(0.25)

    # Set speed to something.
    print('Driving');
    driver.set_drive(R_MTR, FWD, 110)
    driver.set_drive(L_MTR, BWD, 110)

    # Wait.
    time.sleep(1.0)

    # Set speed to zero.
    print('Stopping');
    driver.set_drive(R_MTR, FWD, 0)
    driver.set_drive(L_MTR, FWD, 0)

    # Wait.
    time.sleep(0.25)

    # Set speed to zero.
    print('Disabling');
    driver.disable()


#
#   Main
#
if __name__ == "__main__": 
    # Establish a connection to the motor drivers.
    driver = qwiic_scmd.QwiicScmd()

    # Check.
    if driver.connected == False:
        print("Motor Driver not connected!")
        sys.exit(0)

    # Initialize.
    driver.begin()
    print("Motor driver initialized.")

    try:
        driveBot(driver)
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("Ending due to exception: %s" % repr(exErr))
        driver.disable()
        sys.exit(0)
