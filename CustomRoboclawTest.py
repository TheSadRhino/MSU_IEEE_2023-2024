import time

import roboclaw

addressBack = 0x80
addressFront = 0x81

rc = roboclaw.Roboclaw(addressBack, "/dev/serial0", 115200)
rc.open()

rc.driveM1Forward(64)
time.sleep(3)
rc.driveM1Forward(0)
