import time

from roboclaw import Roboclaw

addressBack = 0x80
addressFront = 0x81

roboclawMultiplexer = Roboclaw("/dev/ttyS0", 115200)

if roboclawMultiplexer.open():
    print(roboclawMultiplexer.driveM1Forward(addressFront, 64))

    time.sleep(1)

    print(roboclawMultiplexer.driveM2Forward(addressFront, 64))

    time.sleep(1)

    print(roboclawMultiplexer.driveM1Forward(addressBack, 64))

    time.sleep(1)

    print(roboclawMultiplexer.driveM2Forward(addressBack, 64))

    time.sleep(1)

    print(roboclawMultiplexer.driveForwardMixedMode(addressFront, 0))
    print(roboclawMultiplexer.driveForwardMixedMode(addressBack, 0))
else:
    print("Couldn't open the serial bus.")
