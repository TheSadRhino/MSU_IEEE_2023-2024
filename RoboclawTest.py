import time

import serial

import roboclaw.bytebuffer

addressBack = 0x80
addressFront = 0x81

serialDevice = serial.Serial(port="/dev/serial0", baudrate=115200)
rc = roboclaw.bytebuffer.Roboclaw(serialDevice)

rc.forward_m1(64)
time.sleep(3)
rc.forward_m1(0)

rc.forward_m2(64)
time.sleep(3)
rc.forward_m2(0)

rc.forward_m1(64, addressFront)
time.sleep(3)
rc.forward_m1(0, addressFront)

rc.forward_m2(64, addressFront)
time.sleep(3)
rc.forward_m2(0, addressFront)



# #roboclawMultiplexer = Roboclaw("/dev/serial0", 115200)
#
# if roboclawMultiplexer.open():
#     print(roboclawMultiplexer.driveM1Forward(addressFront, 64))
#
#     time.sleep(1)
#
#     print(roboclawMultiplexer.driveM2Forward(addressFront, 64))
#
#     time.sleep(1)
#
#     print(roboclawMultiplexer.driveM1Forward(addressBack, 64))
#
#     time.sleep(1)
#
#     print(roboclawMultiplexer.driveM2Forward(addressBack, 64))
#
#     time.sleep(1)
#
#     print(roboclawMultiplexer.driveForwardMixedMode(addressFront, 0))
#     print(roboclawMultiplexer.driveForwardMixedMode(addressBack, 0))
# else:
#     print("Couldn't open the serial bus.")
