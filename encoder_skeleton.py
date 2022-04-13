#!/usr/bin/env python3
#
#   encoder_skeleton.py
#
#   Create an Encoder object to process the encoders: Prepare and
#   handle the GPIO interrupts and count the encoder!
#
import pigpio
import time


#
#   Encoder Object
#
#   This implements the left and right wheel encoder.  NOTE the
#   channels may be different?
#
class Encoder:
    # Initialize.
    def __init__(self, chLA = 24, chLB = 25, chRA = 23, chRB = 22):
        # Initialize the connection to the pigpio daemon.
        self.io = pigpio.pi()
        if not self.io.connected:
            raise Exception("Unable to connection to pigpio daemon!")
        print("Connected to pigpio daemon.")

        # Setup the input pins with a pull-up resistor.  The encoders
        # (Hall Effect chip) are open drain output, i.e. only pull
        # down.  Set up all channels this way:
        for channel in [chLA, chLB, chRA, chRB]:
            print("Setting up input GPIO%2d with pull-up..." % channel)
            self.io.set_mode(channel, pigpio.INPUT)
            self.io.set_pull_up_down(channel, pigpio.PUD_UP)

        # Prepare/initialize the channel states.
        self.LA = self.io.read(chLA)
        self.LB = self.io.read(chLB)
        self.RA = self.io.read(chRA)
        self.RB = self.io.read(chRB)

        # Prepare/initialize the encoder counts.
        self.lcount = 0;
        self.rcount = 0;

        # Finally, prepare the callbacks, setting things up.
        print("Starting the callback functions...")
        self.cbs = [self.io.callback(chLA, pigpio.RISING_EDGE,  self.LArise),
                    self.io.callback(chLB, pigpio.RISING_EDGE,  self.LBrise),
                    self.io.callback(chRA, pigpio.RISING_EDGE,  self.RArise),
                    self.io.callback(chRB, pigpio.RISING_EDGE,  self.RBrise),
                    self.io.callback(chLA, pigpio.FALLING_EDGE, self.LAfall),
                    self.io.callback(chLB, pigpio.FALLING_EDGE, self.LBfall),
                    self.io.callback(chRA, pigpio.FALLING_EDGE, self.RAfall),
                    self.io.callback(chRB, pigpio.FALLING_EDGE, self.RBfall)]

    # Cleanup.
    def shutdown(self):
        # Simply cancel the callback functions.
        print("Cancelling the callback functions...")
        for cb in self.cbs:
            cb.cancel()

        # Wait until all is done, then stop the I/O
        time.sleep(0.25)
        self.io.stop()


    # Report
    def leftencoder(self):
        return (self.lcount)
    def rightencoder(self):
        return (self.rcount)

    # Callback Functions:
    def LArise(self, gpio, level, tick):
        print("LArise");
        # Update
        if (not self.LB):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state
        self.LA = True

    def LAfall(self, gpio, level, tick):
        # Update
        if (self.LB):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state
        self.LA = False

    def LBrise(self, gpio, level, tick):
        print("LBrise");
        # Update
        if (self.LA):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state
        self.LB = True

    def LBfall(self, gpio, level, tick):
        # Update
        if (not self.LA):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state
        self.LB = False

    def RArise(self, gpio, level, tick):
        # Update
        if (not self.RB):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state
        self.RA = True

    def RAfall(self, gpio, level, tick):
        # Update
        if (self.RB):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state
        self.RA = False

    def RBrise(self, gpio, level, tick):
        # Update
        if (self.RA):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state
        self.RB = True

    def RBfall(self, gpio, level, tick):
        # Update
        if (not self.RA):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state
        self.RB = False


#
#   Main
#
if __name__ == "__main__":
    # Initialize the encoder object.
    encoder = Encoder()

    # Loop and read.
    print("Running...")
    try:
        while True:
            print("Encoders:  Left %5d - Right %5d" %
                  (encoder.leftencoder(), encoder.rightencoder()))
            time.sleep(0.05)
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("Ending due to exception: %s" % repr(exErr))

    # Cleanup (shutting down the callbacks).
    encoder.shutdown()
