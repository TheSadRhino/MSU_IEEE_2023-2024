import math
from math import atan2, asin


def threeDimensionalMagnitude(x, y, z):
    return math.sqrt(x**2 + y**2 + z**2)


def quaternionToEuler(qr, qi, qj, qk, degrees: bool = False):
    sqr = qr ** 2
    sqi = qi ** 2
    sqj = qj ** 2
    sqk = qk ** 2

    _yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr))
    if (sqi + sqj + sqk + sqr) != 0:
        _pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr))
    else:
        _pitch = math.copysign(math.pi/2, -2.0 * (qi * qk - qj * qr))
    _roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr))

    if degrees:
        _yaw = math.degrees(_yaw)
        _pitch = math.degrees(_pitch)
        _roll = math.degrees(_roll)

    return _yaw, _pitch, _roll
