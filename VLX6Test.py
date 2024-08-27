# SPDX-FileCopyrightText: 2021 Smankusors for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Example of how to use the adafruit_vl53l0x library to change the assigned address of
multiple VL53L0X sensors on the same I2C bus. This example only focuses on 2 VL53L0X
sensors, but can be modified for more. BE AWARE: a multitude of sensors may require
more current than the on-board 3V regulator can output (typical current consumption during
active range readings is about 19 mA per sensor).

This example like vl53l0x_multiple_sensors, but this with sensors in continuous mode.
So you don't need to wait the sensor to do range measurement and return the distance
for you.

For example, you have 2 VL53L0X sensors, with timing budget of 200ms, on single mode.
When you want to get distance from sensor #1, sensor #2 will idle because waiting
for sensor #1 completes the range measurement. You could do multithreading so you
can ask both the sensor at the same time, but it's quite expensive.

When you use continuous mode, the sensor will always do range measurement after it
completes. So when you want to get the distance from both of the device, you don't
need to wait 400ms, just 200ms for both of the sensors.
"""
import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_vl6180x import VL6180X
import adafruit_tca9548a

if __name__ == "__main__":
    i2c = board.I2C()  # uses board.SCL and board.SDA
    # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

    # Create the TCA9548A object and give it the I2C bus
    tca = adafruit_tca9548a.TCA9548A(i2c)

    # For each sensor, create it using the TCA9548A channel instead of the I2C object
    tsl1 = VL6180X(tca[0])
    tsl2 = VL6180X(tca[1])

    tsl1.start_range_continuous()
    tsl2.start_range_continuous()

    # After initial setup, can just use sensors as normal.
    while True:
        print(time.time_ns(), tsl1.range, tsl2.range)

    tsl1.stop_range_continuous()
    tsl2.stop_range_continuous()
else:
    print(
        "Multiple VL53L0X sensors' addresses are assigned properly\n"
        "execute detect_range() to read each sensors range readings.\n"
        "When you are done with readings, execute stop_continuous()\n"
        "to stop the continuous mode."
    )