import math
import time
from math import atan2, asin

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C


def quaternionToEuler(qr, qi, qj, qk, degrees: bool = False):
    sqr = qr ** 2
    sqi = qi ** 2
    sqj = qj ** 2
    sqk = qk ** 2

    _yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr))
    _pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr))
    _roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr))

    if degrees:
        _yaw = math.degrees(_yaw)
        _pitch = math.degrees(_pitch)
        _roll = math.degrees(_roll)

    return _yaw, _pitch, _roll


i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

while True:
    time.sleep(0.05)
    print("Rotation Vector Quaternion:", end="")
    try:
        quatI, quatJ, quatK, quatReal = bno.quaternion  # pylint:disable=no-member
        print(
            "I: %0.6f  J: %0.6f  K: %0.6f  Real: %0.6f" % (quatI, quatJ, quatK, quatReal)
        )

        yaw, pitch, roll = quaternionToEuler(quatReal, quatI, quatJ, quatK, True)
        print("Yaw: %0.6f  Pitch: %0.6f  Roll: %0.6f" % (yaw, pitch, roll))
    except Exception as e:
        print("Error occurred in Rotation Vector: " + str(e))
