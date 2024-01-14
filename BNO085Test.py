import time

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_RAW_ACCELEROMETER,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_RAW_ACCELEROMETER)

while True:
    time.sleep(0.5)
    print("Gyro:")
    try:
        gyroI, gyroJ, gyroK = bno.gyro

        print(
            "I: %0.6f  J: %0.6f K: %0.6f" % (gyroI, gyroJ, gyroK)
        )
    except Exception as e:
        print("Error occurred in Gyro: " + str(e))
    print("")

    print("Accelerometer:")
    try:
        accelI, accelJ, accelK = bno.acceleration

        print(
            "I: %0.6f  J: %0.6f K: %0.6f" % (accelI, accelJ, accelK)
        )
    except Exception as e:
        print("Error occurred in Accelerometer: " + str(e))
    print("")

    print("Raw Accelerometer:")
    try:
        rawAccelI, rawAccelJ, rawAccelK = bno.raw_acceleration

        print(
            "I: %0.6f  J: %0.6f K: %0.6f" % (rawAccelI, rawAccelJ, rawAccelK)
        )
    except Exception as e:
        print("Error occurred in Raw Accelerometer: " + str(e))
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
