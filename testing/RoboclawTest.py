import time

from roboclaw import Roboclaw

addressBack = 0x80
addressFront = 0x81

roboclawMultiplexer = Roboclaw("/dev/ttyS0", 115200)

if roboclawMultiplexer.open():
    roboclawMultiplexer.driveM1Forward(addressFront, 127)

    time.sleep(1)

    roboclawMultiplexer.driveM2Forward(addressFront, 127)

    time.sleep(1)

    roboclawMultiplexer.driveM1Forward(addressBack, 127)

    time.sleep(1)

    roboclawMultiplexer.driveM2Forward(addressBack, 127)

    time.sleep(1)

    roboclawMultiplexer.driveForwardMixedMode(addressFront, 0)
    roboclawMultiplexer.driveForwardMixedMode(addressBack, 0)
else:
    print("Couldn't open the serial bus.")
