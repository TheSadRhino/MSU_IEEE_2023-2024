import time

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

while True:
    time.sleep(0.5)
    print("Game Rotation Vector Quaternion:")
    try:
        gameI, gameJ, gameK, gameReal = bno.game_quaternion

        print(
            "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (gameI, gameJ, gameK, gameReal)
        )
    except Exception as e:
        print("Error occurred in Game Rotation Vector: " + str(e))
    print("")

    print("Rotation Vector Quaternion:")
    try:
        quatI, quatJ, quatK, quatReal = bno.quaternion  # pylint:disable=no-member
        print(
            "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quatI, quatJ, quatK, quatReal)
        )
    except Exception as e:
        print("Error occurred in Rotation Vector: " + str(e))
    print("")