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
from sensors.VL53l0X import VL53L0X
import RPi.GPIO as GPIO
from adafruit_extended_bus import ExtendedI2C
from sensors.VL6180X import VL6180X


# declare the singleton variable for the default I2C bus
i2c = busio.I2C(board.SCL, board.SDA)
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

GPIO.setmode(GPIO.BCM)

# declare the digital output pins connected to the "SHDN" pin on each VL53L0X sensor
xshut = [
    9,
    25
    # add more VL53L0X sensors by defining their SHDN pins here
]

#GPIO.setup(25, GPIO.OUT)
#GPIO.output(25, False)

GPIO.setup(20, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)

GPIO.output(20, False)
GPIO.output(26, False)
GPIO.output(13, False)
GPIO.output(8, False)
GPIO.output(19, False)
GPIO.output(6, False)

GPIO.output(20, True)
vl5 = VL53L0X(i2c)
vl5.start_continuous()
vl5.set_address(0x31)


#GPIO.output(26, True)
#vl6 = VL6180X(i2c)
#vl6.start_continuous()
#vl6.set_address(0x30)

# there is a helpful list of pre-designated I2C addresses for various I2C devices at
# https://learn.adafruit.com/i2c-addresses/the-list
# According to this list 0x30-0x34 are available, although the list may be incomplete.
# In the python REPR, you can scan for all I2C devices that are attached and detirmine
# their addresses using:
#   >>> import board
#   >>> i2c = board.I2C()  # uses board.SCL and board.SDA
#   >>> if i2c.try_lock():
#   >>>     [hex(x) for x in i2c.scan()]
#   >>>     i2c.unlock()


def detect_range(count=10):
    """take count=5 samples"""
    while count:
        print("Sensor {} Range: {}mm".format(1, vl5.range))
        #print("Sensor {} Range: {}mm".format(2, vl6.range))

        time.sleep(1.0)


def stop_continuous():
    """this is not required, if you use XSHUT to reset the sensor.
    unless if you want to save some energy
    """
    vl5.stop_continuous()
    #vl5.stop_continuous()


if __name__ == "__main__":
    detect_range()
    stop_continuous()
else:
    print(
        "Multiple VL53L0X sensors' addresses are assigned properly\n"
        "execute detect_range() to read each sensors range readings.\n"
        "When you are done with readings, execute stop_continuous()\n"
        "to stop the continuous mode."
    )