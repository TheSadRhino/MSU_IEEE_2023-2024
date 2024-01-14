import time
from enum import Enum

import serial
from serial import SerialException


# Utilities for bit operations
def _32BitValueToBytes(longValue: int) -> [bytes, bytes, bytes, bytes]:
    byte0 = longValue
    byte1 = longValue >> 8
    byte2 = longValue >> 16
    byte3 = longValue >> 24

    return bytes(byte3 & 0xFF), bytes(byte2 & 0xFF), bytes(byte1 & 0xFF), bytes(byte0 & 0xFF)


def _16BitValueToBytes(wordValue: int) -> [bytes, bytes]:
    byte0 = wordValue
    byte1 = wordValue >> 8
    return bytes(byte1 & 0xFF), bytes(byte0 & 0xFF)


def _boolToBytes(value: bool) -> [bytes]:
    return bytes(1) if value else bytes(0),


def _bytesTo16BitValue(byteValues: [bytes, bytes]) -> int:
    if len(byteValues) != 2:
        return -1
    else:
        return byteValues[1] << 8 | byteValues[0]


def _bytesTo32BitValue(byteValues: [bytes, bytes, bytes, bytes]) -> int:
    if len(byteValues) != 4:
        return -1
    else:
        return byteValues[3] << 24 | byteValues[2] << 16 | byteValues[1] << 8 | byteValues[0]


class Roboclaw:
    def __init__(self, comPort, baudRate, packetTimeout=0.01, retries=3, communicationTimeout=1):
        self.__comPort = comPort
        self.__baudRate = baudRate
        self.__packetTimeout = packetTimeout
        self.__communicationTimeout = communicationTimeout
        self.__maxRetries = retries
        self.__port = None
        self.__crc16 = 0

    def open(self) -> bool:
        try:
            self.__port = serial.Serial(port=self.__comPort, baudrate=self.__baudRate,
                                        timeout=self.__communicationTimeout, interCharTimeout=self.__packetTimeout)
        except SerialException as exception:
            print(exception)
            return False
        return True

    class Command(Enum):
        DRIVE_M1_FORWARD = 0
        DRIVE_M1_BACKWARD = 1
        SET_MAIN_VOLTAGE_MINIMUM = 2
        SET_MAIN_VOLTAGE_MAXIMUM = 3
        DRIVE_M2_FORWARD = 4
        DRIVE_M2_BACKWARD = 5
        DRIVE_M1_7BIT = 6
        DRIVE_M2_7BIT = 7
        DRIVE_FORWARD_MIXED_MODE = 8
        DRIVE_BACKWARD_MIXED_MODE = 9
        TURN_RIGHT_MIXED_MODE = 10
        TURN_LEFT_MIXED_MODE = 11
        DRIVE_MIXED_MODE_7BIT = 12
        TURN_MIXED_MODE_7BIT = 13
        SET_SERIAL_TIMEOUT = 14
        READ_SERIAL_TIMEOUT = 15
        READ_M1_ENCODER_COUNT = 16
        READ_M2_ENCODER_COUNTS = 17
        READ_M1_ENCODER_COUNTS_PER_SECOND = 18
        READ_M2_ENCODER_COUNTS_PER_SECOND = 19
        RESET_QUADRATURE_ENCODERS = 20
        READ_FIRMWARE_VERSION = 21
        SET_M1_QUADRATURE_ENCODER_REGISTER = 22
        SET_M2_QUADRATURE_ENCODER_REGISTER = 23
        READ_MAIN_BATTERY_VOLTAGE = 24
        READ_LOGIC_BATTERY_VOLTAGE = 25
        SET_MINIMUM_LOGIC_BATTERY_VOLTAGE = 26
        SET_MAXIMUM_LOGIC_BATTERY_VOLTAGE = 27
        SET_M1_VELOCITY_PID_CONSTANTS = 28
        SET_M2_VELOCITY_PID_CONSTANTS = 29
        READ_M1_INSTANTANEOUS_COUNTS_PER_SECOND = 30
        READ_M2_INSTANTANEOUS_COUNTS_PER_SECOND = 31
        DRIVE_M1_DUTY_CYCLE = 32
        DRIVE_M2_DUTY_CYCLE = 33
        DRIVE_M1M2_DUTY_CYCLE = 34
        # TODO: figure out if this is counts or pulses per second
        DRIVE_M1_PULSES_PER_SECOND = 35
        DRIVE_M2_PULSES_PER_SECOND = 36
        DRIVE_M1M2_PULSES_PER_SECOND = 37
        DRIVE_M1_PULSES_PER_SECOND_WITH_ACCELERATION = 38
        DRIVE_M2_PULSES_PER_SECOND_WITH_ACCELERATION = 39
        DRIVE_M1M2_PULSES_PER_SECOND_WITH_ACCELERATION = 40
        DRIVE_M1_PULSES_PER_SECOND_FOR_DISTANCE_BUFFERED = 41
        DRIVE_M2_PULSES_PER_SECOND_FOR_DISTANCE_BUFFERED = 42
        DRIVE_M1M2_PULSES_PER_SECOND_FOR_DISTANCE_BUFFERED = 43
        DRIVE_M1_PULSES_PER_SECOND_WITH_ACCELERATION_FOR_DISTANCE_BUFFERED = 44
        DRIVE_M2_PULSES_PER_SECOND_WITH_ACCELERATION_FOR_DISTANCE_BUFFERED = 45
        DRIVE_M1M2_PULSES_PER_SECOND_WITH_ACCELERATION_FOR_DISTANCE_BUFFERED = 46
        READ_M1M2_BUFFER_LENGTH = 47
        READ_M1M2_PWM_VALUES = 48
        READ_M1M2_CURRENT_DRAW = 49
        DRIVE_M1M2_PULSES_PER_SECOND_WITH_INDIVIDUAL_ACCELERATION = 50
        DRIVE_M1M2_PULSES_PER_SECOND_WITH_INDIVIDUAL_ACCELERATION_FOR_DISTANCE_BUFFERED = 51
        DRIVE_M1_DUTY_CYCLE_WITH_ACCELERATION = 52
        DRIVE_M2_DUTY_CYCLE_WITH_ACCELERATION = 53
        DRIVE_M1M2_DUTY_CYCLE_WITH_ACCELERATION = 54
        READ_M1_VELOCITY_PID_COEFFICIENTS_AND_ACCELERATION_SETTINGS = 55
        READ_M2_VELOCITY_PID_COEFFICIENTS_AND_ACCELERATION_SETTINGS = 56
        SET_MAIN_BATTERY_VOLTAGES = 57
        SET_LOGIC_BATTERY_VOLTAGES = 58
        READ_MAIN_BATTERY_SETTINGS = 59
        READ_LOGIC_BATTERY_SETTINGS = 60
        SET_M1_POSITION_PID_COEFFICIENTS = 61
        SET_M2_POSITION_PID_COEFFICIENTS = 62
        READ_M1_POSITION_PID_COEFFICIENTS = 63
        READ_M2_POSITION_PID_COEFFICIENTS = 64
        DRIVE_M1_TO_POSITION_WITH_VELOCITY_ACCELERATION_AND_DECELERATION_BUFFERED = 65
        DRIVE_M2_TO_POSITION_WITH_VELOCITY_ACCELERATION_AND_DECELERATION_BUFFERED = 66
        DRIVE_M1M2_TO_POSITION_WITH_VELOCITY_ACCELERATION_AND_DECELERATION_BUFFERED = 67
        SET_M1_DEFAULT_ACCELERATION_FOR_DUTY_CYCLE = 68
        SET_M2_DEFAULT_ACCELERATION_FOR_DUTY_CYCLE = 69
        SET_M1_DEFAULT_VELOCITY_FOR_POSITION = 70
        SET_M2_DEFAULT_VELOCITY_FOR_POSITION = 71
        READ_M1M2_DEFAULT_VELOCITY_SETTINGS = 72

        SET_S3S4S5_MODES = 74
        READ_S3S4S5_MODES = 75
        SET_RC_ANALOG_DEADBAND = 76
        READ_RC_ANALOG_DEADBAND = 77
        READ_M1M2_ENCODER_COUNTS = 78
        READ_M1M2_INSTANTANEOUS_ENCODER_COUNTS_PER_SECOND = 79
        RESTORE_FACTORY_SETTINGS = 80
        READ_DEFAULT_ACCELERATION_SETTINGS_FOR_DUTY_CYCLE = 81
        READ_BOARD_TEMPERATURE = 82
        READ_SECOND_BOARD_TEMPERATURE = 83

        READ_STATUS = 90
        READ_M1M2_ENCODER_MODE = 91
        SET_M1_ENCODER_MODE = 92
        SET_M2_ENCODER_MODE = 93
        WRITE_SETTINGS_TO_EEPROM = 94
        READ_SETTINGS_FROM_EEPROM = 95

        SET_STANDARD_CONFIGURATION_SETTINGS = 98
        READ_STANDARD_CONFIGURATION_SETTINGS = 99
        SET_CTRL_PIN_MODES = 100
        READ_CTRL_PIN_MODES = 101
        SET_CTRL1 = 102
        SET_CTRL2 = 103
        READ_CTRL1CTRL2_VALUES = 104
        SET_M1_HOMING_DUTY_OR_SPEED_AND_TIMEOUT = 105
        SET_M2_HOMING_DUTY_OR_SPEED_AND_TIMEOUT = 106
        READ_M1M2_HOMING_SETTINGS = 107
        READ_M1M2_AVERAGE_ENCODER_COUNTS_PER_SECOND = 108
        SET_M1M2_VELOCITY_ERROR_LIMITS = 109
        READ_M1M2_VELOCITY_ERROR_LIMITS = 110
        READ_M1M2_CALCULATED_ENCODER_SPEED_ERROR = 111
        SET_M1M2_POSITION_ERROR_LIMITS = 112
        READ_M1M2_POSITION_ERROR_LIMITS = 113
        READ_M1M2_CALCULATED_POSITION_ERROR = 114
        SET_BATTERY_OFFSET_VOLTAGES = 115
        READ_BATTERY_OFFSET_VOLTAGES = 116
        SET_CURRENT_BLANKING_PERCENTAGES = 117
        READ_CURRENT_BLANKING_PERCENTAGES = 118
        DRIVE_M1_TO_POSITION_BUFFERED = 119
        DRIVE_M2_TO_POSITION_BUFFERED = 120
        DRIVE_M1M2_TO_POSITION_BUFFERED = 121
        DRIVE_M1_TO_POSITION_WITH_VELOCITY_BUFFERED = 122
        DRIVE_M2_TO_POSITION_WITH_VELOCITY_BUFFERED = 123
        DRIVE_M1M2_TO_POSITION_WITH_VELOCITY_BUFFERED = 124

        SET_M1_CURRENT_LIMIT = 133
        SET_M2_CURRENT_LIMIT = 134
        READ_M1_CURRENT_LIMIT = 135
        READ_M2_CURRENT_LIMIT = 136

        SET_PWM_MODE = 148
        GET_PWM_MODE = 149

        WRITE_USER_EEPROM_MEMORY_LOCATION = 252
        READ_USER_EEPROM_MEMORY_LOCATION = 253

        FLAG_BOOTLOADER = 255

    class S3Mode(Enum):
        DEFAULT = 0x00
        EMERGENCY_STOP = 0x01
        EMERGENCY_STOP_LATCHING = 0x81
        VOLTAGE_CLAMP = 0x14
        RS485_DIRECTION = 0x24
        ENCODER_TOGGLE = 0x84

    class S4Mode(Enum):
        DISABLED = 0x00
        EMERGENCY_STOP = 0x01
        EMERGENCY_STOP_LATCHING = 0x81
        VOLTAGE_CLAMP = 0x14
        BRAKE = 0x04
        HOME_AUTO = 0xE2
        HOME_USER = 0x62
        HOME_AUTO_OR_LIMIT_FORWARD = 0xF2
        HOME_USER_OR_LIMIT_FORWARD = 0x72
        LIMIT_FORWARD = 0x12
        LIMIT_BACKWARD = 0x22
        LIMIT_BOTH = 0x32

    class S5Mode(Enum):
        DISABLED = 0x00
        EMERGENCY_STOP = 0x01
        EMERGENCY_STOP_LATCHING = 0x81
        VOLTAGE_CLAMP = 0x14
        BRAKE = 0x04
        HOME_USER = 0x62
        HOME_AUTO_OR_LIMIT_FORWARD = 0xF2
        HOME_USER_OR_LIMIT_FORWARD = 0x72
        LIMIT_FORWARD = 0x12
        LIMIT_BACKWARD = 0x22
        LIMIT_BOTH = 0x32

    class Status(Enum):
        NORMAL = 0x000000
        EMERGENCY_STOP = 0x000001
        TEMPERATURE_ERROR = 0x000002
        TEMPERATURE_2_ERROR = 0x0000004
        MAIN_VOLTAGE_HIGH_ERROR = 0x000008
        LOGIC_VOLTAGE_HIGH_ERROR = 0x000010
        LOGIC_VOLTAGE_LOW_ERROR = 0x000020
        M1_DRIVER_FAULT_ERROR = 0x000040
        M2_DRIVER_FAULT_ERROR = 0x000080
        M1_SPEED_ERROR = 0x000100
        M2_SPEED_ERROR = 0x000200
        M1_POSITION_ERROR = 0x000400
        M2_POSITION_ERROR = 0x000800
        M1_CURRENT_ERROR = 0x001000
        M2_CURRENT_ERROR = 0x002000
        M1_OVER_CURRENT_WARNING = 0x010000
        M2_OVER_CURRENT_WARNING = 0x020000
        MAIN_VOLTAGE_HIGH_WARNING = 0x040000
        MAIN_VOLTAGE_LOW_WARNING = 0x080000
        TEMPERATURE_WARNING = 0x100000
        TEMPERATURE_2_WARNING = 0x200000
        S4_SIGNAL_TRIGGERED = 0x400000
        S5_SIGNAL_TRIGGERED = 0x800000
        SPEED_ERROR_LIMIT_WARNING = 0x01000000
        POSITION_ERROR_LIMIT_WARNING = 0x02000000

    class ConfigurationSettings:
        class Mode(Enum):
            RC_MODE = 0x0000
            ANALOG_MODE = 0x0001
            SIMPLE_SERIAL_MODE = 0x0002
            PACKET_SERIAL_MODE = 0x0003

            # @staticmethod
            # def getFromBytes(byte: int) -> roboclaw.roboclaw.Roboclaw.ConfigurationSettings.Mode:
            #     for mode in roboclaw.roboclaw.Roboclaw.ConfigurationSettings.Mode:
            #         if byte & mode.value:
            #             return mode

        class BatteryMode(Enum):
            BATTERY_MODE_OFF = 0x0000
            BATTERY_MODE_AUTO = 0x0004
            BATTERY_MODE_TWO_CELL = 0x0008
            BATTERY_MODE_THREE_CELL = 0x000C
            BATTERY_MODE_FOUR_CELL = 0x0010
            BATTERY_MODE_FIVE_CELL = 0x0014
            BATTERY_MODE_SIX_CELL = 0x0018
            BATTERY_MODE_SEVEN_CELL = 0x001C

            # @staticmethod
            # def getFromBytes(byte: int) -> roboclaw.roboclaw.Roboclaw.ConfigurationSettings.BatteryMode:
            #     for batteryMode in roboclaw.roboclaw.Roboclaw.ConfigurationSettings.BatteryMode:
            #         if byte & batteryMode.value:
            #             return batteryMode

        class RCAnalogSettings:
            MIXING = 0x0020
            EXPONENTIAL = 0x0040
            MCU = 0x0080
            FLIP_SWITCH = 0x0100

            def __init__(self, mixingOn: bool, exponentialOn: bool, mcuOn: bool, flipSwitchOn: bool):
                self.__mixingOn = mixingOn
                self.__exponentialOn = exponentialOn
                self.__mcuOn = mcuOn
                self.__flipSwitchOn = flipSwitchOn

        class BaudRate(Enum):
            BAUD_RATE_2400 = 0x0000
            BAUD_RATE_9600 = 0x0020
            BAUD_RATE_19200 = 0x0040
            BAUD_RATE_38400 = 0x0060
            BAUD_RATE_57600 = 0x0080
            BAUD_RATE_115200 = 0x00A0
            BAUD_RATE_230400 = 0x00C0
            BAUD_RATE_460800 = 0x00E0

            # @staticmethod
            # def getFromBytes(byte: int) -> roboclaw.roboclaw.Roboclaw.ConfigurationSettings.BaudRate:
            #     for baudRate in roboclaw.roboclaw.Roboclaw.ConfigurationSettings.BaudRate:
            #         if byte & baudRate.value:
            #             return baudRate

        class PacketAddress(Enum):
            PACKET_ADDRESS_0x80 = 0x0000
            PACKET_ADDRESS_0x81 = 0x0100
            PACKET_ADDRESS_0x82 = 0x0200
            PACKET_ADDRESS_0x83 = 0x0300
            PACKET_ADDRESS_0x84 = 0x0400
            PACKET_ADDRESS_0x85 = 0x0500
            PACKET_ADDRESS_0x86 = 0x0600
            PACKET_ADDRESS_0x87 = 0x0700

            # @staticmethod
            # def getFromBytes(byte: int) -> roboclaw.roboclaw.Roboclaw.ConfigurationSettings.PacketAddress:
            #     for packetAddress in roboclaw.roboclaw.Roboclaw.ConfigurationSettings.PacketAddress:
            #         if byte & packetAddress.value:
            #             return packetAddress

        class UtilitySettings:
            SLAVE_MODE = 0x0800
            RELAY_MODE = 0x1000
            SWAP_ENCODERS = 0x2000
            SWAP_BUTTONS = 0x4000
            MULTI_UNIT_MODE = 0x8000

        def __init__(self, mode: Mode, batteryMode: BatteryMode, baudRate: BaudRate, packetAddress: PacketAddress,
                     rcAnalogSettings: RCAnalogSettings, utilitySettings: UtilitySettings):
            self.__mode = mode
            self.__batteryMode = batteryMode
            self.__baudRate = baudRate
            self.__packetAddress = packetAddress
            self.__rcAnalogSettings = rcAnalogSettings
            self.__utilitySettings = utilitySettings

    class CTRLModes(Enum):
        DISABLE = 0
        USER = 1
        VOLTAGE_CLAMP = 2
        BRAKE = 3

    class EncoderMode:
        def __init__(self, encoderIsAbsolute: bool = False, reverseMotorRelativeDirection: bool = False,
                     reverseEncoderRelativeDirection: bool = False, enableRCAnalogEncoderSupport: bool = False):
            self.__encoderIsAbsolute = encoderIsAbsolute
            self.__reverseMotorRelativeDirection = reverseMotorRelativeDirection
            self.__reverseEncoderRelativeDirection = reverseEncoderRelativeDirection
            self.__enableRCAnalogEncoderSupport = enableRCAnalogEncoderSupport

        def setFromByte(self, byte: int) -> None:
            self.__encoderIsAbsolute = byte & 0b0
            self.__reverseEncoderRelativeDirection = byte & 0b100000
            self.__reverseMotorRelativeDirection = byte & 0b1000000
            self.__enableRCAnalogEncoderSupport = byte & 0b10000000

        def getByte(self) -> int:
            output = 0x00
            output |= (1 if self.__encoderIsAbsolute else 0) << 0
            output |= (1 if self.__reverseEncoderRelativeDirection else 0) << 5
            output |= (1 if self.__reverseMotorRelativeDirection else 0) << 6
            output |= (1 if self.__enableRCAnalogEncoderSupport else 0) << 7

            return output

    # CRC16 Calculation Functions
    def _clearCRC16(self) -> None:
        self.__crc16 = 0

    def _updateCRC16SingleByte(self, byte: bytes) -> None:
        self.__crc16 = self.__crc16 ^ ((int.from_bytes(byte, "big") & 0xFF) << 8)
        for bit in range(0, 8):
            if bool(self.__crc16 & 0x8000):
                self.__crc16 = (self.__crc16 << 1) ^ 0x1021
            else:
                self.__crc16 = self.__crc16 << 1

    def _updateCRC16(self, byteArray: [bytes]) -> None:
        for byte in byteArray:
            self._updateCRC16SingleByte(byte)

    def _writeCRC16WriteOnly(self) -> bool:
        byteValues = _16BitValueToBytes(self.__crc16 & 0xFFFF)

        for byte in byteValues:
            self.__port.write(byte)

        return self._isAcknowledged()

    def _writeCRC16AndReadData(self, dataLength: int) -> [bool, [bytes]]:
        byteValues = _16BitValueToBytes(self.__crc16 & 0xFFFF)

        for byte in byteValues:
            self.__port.write(byte)

        data = self.__port.read(dataLength)

        crcValid, crc16 = self._readCRC16()
        if crcValid:
            if crc16 & 0xFFFF == self.__crc16 & 0xFFFF:
                return True, data

        return False, []

    def _readCRC16(self) -> [bool, int]:
        data = self.__port.read(2)
        crc16 = _bytesTo16BitValue(data)

        if crc16 != -1:
            return True, crc16
        return False, 0

    # Ensures packets sent are acknowledged (0xFF sent from device).
    # This allows for retries in the case of bad data sent or received.
    def _isAcknowledged(self) -> bool:
        data = self.__port.read(1)

        if len(data) > 0:
            return bool(data[0] & 0xFF)
        return False

    def _writeAddressAndCommand(self, address: int, commandType: Command) -> None:
        self._updateCRC16SingleByte(bytes(address))
        self._writeSingleByte(bytes(address))
        self._updateCRC16SingleByte(bytes(commandType.value))
        self._writeSingleByte(bytes(commandType.value))

    def _writeSingleByte(self, byte: bytes) -> None:
        self.__port.write(byte)

    def _writeData(self, data: [bytes]) -> None:
        for bit in data:
            self._writeSingleByte(bit)

    def _sendSetCommand(self, address: int, commandType: Command, data: [bytes]) -> bool:
        for _ in range(0, self.__maxRetries):
            self.__port.flushInput()

            self._clearCRC16()
            self._writeAddressAndCommand(address, commandType)

            self._updateCRC16(data)
            self._writeData(data)

            if self._writeCRC16WriteOnly():
                return True
        return False

    def _sendReadCommand(self, address: int,
                         commandType: Command, data: [int], returnedDataLength: int) -> [bool, [int]]:
        for _ in range(0, self.__maxRetries):
            self.__port.flushInput()

            self._clearCRC16()
            self._writeAddressAndCommand(address, commandType)

            self._updateCRC16(data)
            self._writeData(data)

            dataConfirmed, data = self._writeCRC16AndReadData(returnedDataLength)
            if dataConfirmed:
                return True, data
        return False, []

    # Publicly accessible motion commands
    def driveM1Forward(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.DRIVE_M1_FORWARD, [bytes(value & 0xFF)])

    def driveM1Backward(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.DRIVE_M1_BACKWARD, [bytes(value & 0xFF)])

    def driveM2Forward(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.DRIVE_M2_FORWARD, [bytes(value & 0xFF)])

    def driveM2Backward(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.DRIVE_M2_BACKWARD, [bytes(value & 0xFF)])

    def driveM17Bit(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.DRIVE_M1_7BIT, [bytes(value & 0xFF)])

    def driveM27Bit(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.DRIVE_M2_7BIT, [bytes(value & 0xFF)])

    def driveForwardMixedMode(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.DRIVE_FORWARD_MIXED_MODE, [bytes(value & 0xFF)])

    def driveBackwardMixedMode(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.DRIVE_BACKWARD_MIXED_MODE, [bytes(value & 0xFF)])

    def turnRightMixed(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.TURN_RIGHT_MIXED_MODE, [bytes(value & 0xFF)])

    def turnLeftMixed(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.TURN_LEFT_MIXED_MODE, [bytes(value & 0xFF)])

    def driveMixedMode7Bit(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.DRIVE_MIXED_MODE_7BIT, [bytes(value & 0xFF)])

    def turnMixedMode7Bit(self, address: int, value: int) -> bool:
        return self._sendSetCommand(address, self.Command.TURN_MIXED_MODE_7BIT, [bytes(value & 0xFF)])

    def driveM1DutyCycle(self, address: int, duty: int) -> bool:
        byteValues = _16BitValueToBytes(duty)
        return self._sendSetCommand(address, self.Command.DRIVE_M1_DUTY_CYCLE, byteValues)

    def driveM2DutyCycle(self, address: int, duty: int) -> bool:
        byteValues = _16BitValueToBytes(duty)
        return self._sendSetCommand(address, self.Command.DRIVE_M2_DUTY_CYCLE, byteValues)

    def driveM1M2DutyCycle(self, address: int, dutyM1: int, dutyM2: int) -> bool:
        m1ByteValues = _16BitValueToBytes(dutyM1)
        m2ByteValues = _16BitValueToBytes(dutyM2)
        return self._sendSetCommand(address, self.Command.DRIVE_M1M2_DUTY_CYCLE, m1ByteValues + m2ByteValues)

    def driveM1Velocity(self, address: int, velocity: int) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        return self._sendSetCommand(address, self.Command.DRIVE_M1_PULSES_PER_SECOND, velocityByteValues)

    def driveM2Velocity(self, address: int, velocity: int) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        return self._sendSetCommand(address, self.Command.DRIVE_M2_PULSES_PER_SECOND, velocityByteValues)

    def driveM1M2Velocity(self, address: int, velocityM1: int, velocityM2: int) -> bool:
        velocityM1ByteValues = _32BitValueToBytes(velocityM1)
        velocityM2ByteValues = _32BitValueToBytes(velocityM2)
        return self._sendSetCommand(address, self.Command.DRIVE_M1M2_PULSES_PER_SECOND,
                                    velocityM1ByteValues + velocityM2ByteValues)

    def driveM1VelocityAndAcceleration(self, address: int, velocity: int, acceleration: int) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        accelerationByteValues = _32BitValueToBytes(acceleration)
        return self._sendSetCommand(address, self.Command.DRIVE_M1_PULSES_PER_SECOND_WITH_ACCELERATION,
                                    accelerationByteValues + velocityByteValues)

    def driveM2VelocityAndAcceleration(self, address: int, velocity: int, acceleration: int) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        accelerationByteValues = _32BitValueToBytes(acceleration)
        return self._sendSetCommand(address, self.Command.DRIVE_M2_PULSES_PER_SECOND_WITH_ACCELERATION,
                                    accelerationByteValues + velocityByteValues)

    def driveM1M2VelocityAndAcceleration(self, address: int, velocityM1: int,
                                         velocityM2: int, acceleration: int) -> bool:
        velocityM1ByteValues = _32BitValueToBytes(velocityM1)
        velocityM2ByteValues = _32BitValueToBytes(velocityM2)
        accelerationByteValues = _32BitValueToBytes(acceleration)
        return self._sendSetCommand(address, self.Command.DRIVE_M1M2_PULSES_PER_SECOND_WITH_ACCELERATION,
                                    accelerationByteValues + velocityM1ByteValues + velocityM2ByteValues)

    def driveM1VelocityForDistanceBuffered(self, address: int, velocity: int, distance: int, resetBuffer: bool) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        distanceByteValues = _32BitValueToBytes(distance)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.DRIVE_M1_PULSES_PER_SECOND_FOR_DISTANCE_BUFFERED,
                                    velocityByteValues + distanceByteValues + bufferByteValues)

    def driveM2VelocityForDistanceBuffered(self, address: int, velocity: int, distance: int, resetBuffer: bool) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        distanceByteValues = _32BitValueToBytes(distance)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.DRIVE_M2_PULSES_PER_SECOND_FOR_DISTANCE_BUFFERED,
                                    velocityByteValues + distanceByteValues + bufferByteValues)

    def driveM1M2VelocityForDistanceBuffered(self, address: int, velocityM1: int, distanceM1: int,
                                             velocityM2: int, distanceM2: int, resetBuffer: bool) -> bool:
        velocityM1ByteValues = _32BitValueToBytes(velocityM1)
        distanceM1ByteValues = _32BitValueToBytes(distanceM1)
        velocityM2ByteValues = _32BitValueToBytes(velocityM2)
        distanceM2ByteValues = _32BitValueToBytes(distanceM2)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.DRIVE_M1M2_PULSES_PER_SECOND_FOR_DISTANCE_BUFFERED,
                                    velocityM1ByteValues + distanceM1ByteValues +
                                    velocityM2ByteValues + distanceM2ByteValues + bufferByteValues)

    def driveM1VelocityWithAccelerationForDistanceBuffered(self, address: int, velocity: int,
                                                           acceleration: int, distance: int, resetBuffer: bool) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        accelerationByteValues = _32BitValueToBytes(acceleration)
        distanceByteValues = _32BitValueToBytes(distance)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address,
                                    self.Command.DRIVE_M1_PULSES_PER_SECOND_WITH_ACCELERATION_FOR_DISTANCE_BUFFERED,
                                    accelerationByteValues + velocityByteValues + distanceByteValues + bufferByteValues)

    def driveM2VelocityWithAccelerationForDistanceBuffered(self, address: int, velocity: int,
                                                           acceleration: int, distance: int, resetBuffer: bool) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        accelerationByteValues = _32BitValueToBytes(acceleration)
        distanceByteValues = _32BitValueToBytes(distance)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address,
                                    self.Command.DRIVE_M2_PULSES_PER_SECOND_WITH_ACCELERATION_FOR_DISTANCE_BUFFERED,
                                    accelerationByteValues + velocityByteValues + distanceByteValues + bufferByteValues)

    def driveM1M2VelocityWithAccelerationForDistanceBuffered(self, address: int, velocityM1: int, distanceM1: int,
                                                             velocityM2: int, distanceM2: int, acceleration: int,
                                                             resetBuffer: bool) -> bool:
        velocityM1ByteValues = _32BitValueToBytes(velocityM1)
        distanceM1ByteValues = _32BitValueToBytes(distanceM1)
        velocityM2ByteValues = _32BitValueToBytes(velocityM2)
        distanceM2ByteValues = _32BitValueToBytes(distanceM2)
        accelerationByteValues = _32BitValueToBytes(acceleration)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address,
                                    self.Command.DRIVE_M1M2_PULSES_PER_SECOND_WITH_ACCELERATION_FOR_DISTANCE_BUFFERED,
                                    accelerationByteValues + velocityM1ByteValues + distanceM1ByteValues +
                                    velocityM2ByteValues + distanceM2ByteValues + bufferByteValues)

    def driveM1M2VelocityAndIndividualAcceleration(self, address: int, accelerationM1: int, velocityM1: int,
                                                   accelerationM2: int, velocityM2: int) -> bool:
        velocityM1ByteValues = _32BitValueToBytes(velocityM1)
        accelerationM1ByteValues = _32BitValueToBytes(accelerationM1)
        velocityM2ByteValues = _32BitValueToBytes(velocityM2)
        accelerationM2ByteValues = _32BitValueToBytes(accelerationM2)
        return self._sendSetCommand(address, self.Command.DRIVE_M1M2_PULSES_PER_SECOND_WITH_INDIVIDUAL_ACCELERATION,
                                    accelerationM1ByteValues + velocityM1ByteValues + accelerationM2ByteValues +
                                    velocityM2ByteValues)

    def driveM1M2VelocityAndIndividualForDistanceAcceleration(self, address: int, accelerationM1: int, velocityM1: int,
                                                              distanceM1: int, accelerationM2: int, velocityM2: int,
                                                              distanceM2: int) -> bool:
        velocityM1ByteValues = _32BitValueToBytes(velocityM1)
        accelerationM1ByteValues = _32BitValueToBytes(accelerationM1)
        distanceM1ByteValues = _32BitValueToBytes(distanceM1)
        velocityM2ByteValues = _32BitValueToBytes(velocityM2)
        accelerationM2ByteValues = _32BitValueToBytes(accelerationM2)
        distanceM2ByteValues = _32BitValueToBytes(distanceM2)
        return self._sendSetCommand(address, self.Command.
                                    DRIVE_M1M2_PULSES_PER_SECOND_WITH_INDIVIDUAL_ACCELERATION_FOR_DISTANCE_BUFFERED,
                                    accelerationM1ByteValues + velocityM1ByteValues + distanceM1ByteValues +
                                    accelerationM2ByteValues + velocityM2ByteValues + distanceM2ByteValues)

    def driveM1DutyCycleAndAcceleration(self, address: int, acceleration: int, duty: int) -> bool:
        accelerationByteValues = _16BitValueToBytes(acceleration)
        dutyByteValues = _16BitValueToBytes(duty)
        return self._sendSetCommand(address, self.Command.DRIVE_M1_DUTY_CYCLE_WITH_ACCELERATION,
                                    dutyByteValues + accelerationByteValues)

    def driveM2DutyCycleAndAcceleration(self, address: int, acceleration: int, duty: int) -> bool:
        accelerationByteValues = _16BitValueToBytes(acceleration)
        dutyByteValues = _16BitValueToBytes(duty)
        return self._sendSetCommand(address, self.Command.DRIVE_M2_DUTY_CYCLE_WITH_ACCELERATION,
                                    dutyByteValues + accelerationByteValues)

    def driveM1M2DutyCycleAndAcceleration(self, address: int, dutyM1: int, accelerationM1: int,
                                          dutyM2: int, accelerationM2: int) -> bool:
        dutyM1ByteValues = _16BitValueToBytes(dutyM1)
        dutyM2ByteValues = _16BitValueToBytes(dutyM2)
        accelerationM1ByteValues = _16BitValueToBytes(accelerationM1)
        accelerationM2ByteValues = _16BitValueToBytes(accelerationM2)
        return self._sendSetCommand(address, self.Command.DRIVE_M1M2_DUTY_CYCLE_WITH_ACCELERATION,
                                    dutyM1ByteValues + accelerationM1ByteValues +
                                    dutyM2ByteValues + accelerationM2ByteValues)

    def driveM1VelocityWithAccelerationAndDecelerationToPositionBuffered(self, address: int, velocity: int,
                                                                         position: int, acceleration: int,
                                                                         deceleration: int, resetBuffer: bool) -> bool:
        accelerationByteValues = _32BitValueToBytes(acceleration)
        decelerationByteValues = _32BitValueToBytes(deceleration)
        velocityByteValues = _32BitValueToBytes(velocity)
        positionByteValues = _32BitValueToBytes(position)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.
                                    DRIVE_M1_TO_POSITION_WITH_VELOCITY_ACCELERATION_AND_DECELERATION_BUFFERED,
                                    accelerationByteValues + velocityByteValues + decelerationByteValues +
                                    positionByteValues + bufferByteValues)

    def driveM2VelocityWithAccelerationAndDecelerationToPositionBuffered(self, address: int, velocity: int,
                                                                         position: int, acceleration: int,
                                                                         deceleration: int, resetBuffer: bool) -> bool:
        accelerationByteValues = _32BitValueToBytes(acceleration)
        decelerationByteValues = _32BitValueToBytes(deceleration)
        velocityByteValues = _32BitValueToBytes(velocity)
        positionByteValues = _32BitValueToBytes(position)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.
                                    DRIVE_M2_TO_POSITION_WITH_VELOCITY_ACCELERATION_AND_DECELERATION_BUFFERED,
                                    accelerationByteValues + velocityByteValues + decelerationByteValues +
                                    positionByteValues + bufferByteValues)

    def driveM1M2VelocityWithAccelerationAndDecelerationToPositionBuffered(self, address: int, velocityM1: int,
                                                                           positionM1: int, accelerationM1: int,
                                                                           decelerationM1: int, velocityM2: int,
                                                                           positionM2: int, accelerationM2: int,
                                                                           decelerationM2: int,
                                                                           resetBuffer: bool) -> bool:
        accelerationM1ByteValues = _32BitValueToBytes(accelerationM1)
        decelerationM1ByteValues = _32BitValueToBytes(decelerationM1)
        velocityM1ByteValues = _32BitValueToBytes(velocityM1)
        positionM1ByteValues = _32BitValueToBytes(positionM1)
        accelerationM2ByteValues = _32BitValueToBytes(accelerationM2)
        decelerationM2ByteValues = _32BitValueToBytes(decelerationM2)
        velocityM2ByteValues = _32BitValueToBytes(velocityM2)
        positionM2ByteValues = _32BitValueToBytes(positionM2)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.
                                    DRIVE_M1M2_TO_POSITION_WITH_VELOCITY_ACCELERATION_AND_DECELERATION_BUFFERED,
                                    accelerationM1ByteValues + velocityM1ByteValues + decelerationM1ByteValues +
                                    positionM1ByteValues + accelerationM2ByteValues + velocityM2ByteValues +
                                    decelerationM2ByteValues + positionM2ByteValues + bufferByteValues)

    def driveM1ToPositionBuffered(self, address: int, position: int, resetBuffer: bool) -> bool:
        positionByteValues = _32BitValueToBytes(position)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.DRIVE_M1_TO_POSITION_BUFFERED,
                                    positionByteValues + bufferByteValues)

    def driveM2ToPositionBuffered(self, address: int, position: int, resetBuffer: bool) -> bool:
        positionByteValues = _32BitValueToBytes(position)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.DRIVE_M2_TO_POSITION_BUFFERED,
                                    positionByteValues + bufferByteValues)

    def driveM1M2ToPositionBuffered(self, address: int, positionM1: int, positionM2: int, resetBuffer: bool) -> bool:
        positionM1ByteValues = _32BitValueToBytes(positionM1)
        positionM2ByteValues = _32BitValueToBytes(positionM2)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.DRIVE_M1M2_TO_POSITION_BUFFERED,
                                    positionM1ByteValues + positionM2ByteValues + bufferByteValues)

    def driveM1VelocityToPositionBuffered(self, address: int, velocity: int, position: int, resetBuffer: bool) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        positionByteValues = _32BitValueToBytes(position)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.DRIVE_M1_TO_POSITION_WITH_VELOCITY_BUFFERED,
                                    velocityByteValues + positionByteValues + bufferByteValues)

    def driveM2VelocityToPositionBuffered(self, address: int, velocity: int, position: int, resetBuffer: bool) -> bool:
        velocityByteValues = _32BitValueToBytes(velocity)
        positionByteValues = _32BitValueToBytes(position)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.DRIVE_M2_TO_POSITION_WITH_VELOCITY_BUFFERED,
                                    velocityByteValues + positionByteValues + bufferByteValues)

    def driveM1M2VelocityToPositionBuffered(self, address: int, velocityM1: int, positionM1: int,
                                            velocityM2: int, positionM2: int, resetBuffer: bool) -> bool:
        velocityM1ByteValues = _32BitValueToBytes(velocityM1)
        positionM1ByteValues = _32BitValueToBytes(positionM1)
        velocityM2ByteValues = _32BitValueToBytes(velocityM2)
        positionM2ByteValues = _32BitValueToBytes(positionM2)
        bufferByteValues = _boolToBytes(resetBuffer)
        return self._sendSetCommand(address, self.Command.DRIVE_M1M2_TO_POSITION_WITH_VELOCITY_BUFFERED,
                                    velocityM1ByteValues + positionM1ByteValues +
                                    velocityM2ByteValues + positionM2ByteValues + bufferByteValues)

    # Publicly accessible settings write commands
    def setSerialTimeout(self, address: int, value: float) -> bool:
        timeout = int(value*10)
        return self._sendSetCommand(address, self.Command.SET_SERIAL_TIMEOUT, [timeout])

    def setMainVoltageMinimum(self, address: int, value: float) -> bool:
        voltage = int((value - 6) * 5)
        return self._sendSetCommand(address, self.Command.SET_MAIN_VOLTAGE_MINIMUM, [voltage])

    def setMainVoltageMaximum(self, address: int, value: float) -> bool:
        voltage = int(value*5.12)
        return self._sendSetCommand(address, self.Command.SET_MAIN_VOLTAGE_MAXIMUM, [voltage])

    def setLogicBatteryVoltageMinimum(self, address: int, value: int) -> bool:
        voltage = int((value - 6) * 5)
        return self._sendSetCommand(address, self.Command.SET_MINIMUM_LOGIC_BATTERY_VOLTAGE, [voltage])

    def setLogicBatteryVoltageMaximum(self, address: int, value: int) -> bool:
        voltage = int(value*5.12)
        return self._sendSetCommand(address, self.Command.SET_MAXIMUM_LOGIC_BATTERY_VOLTAGE, [voltage])

    def setMainVoltageLimits(self, address: int, minimumVoltage: float, maximumVoltage: float) -> bool:
        minimumBytes = _16BitValueToBytes(int(minimumVoltage*10))
        maximumBytes = _16BitValueToBytes(int(maximumVoltage*10))
        return self._sendSetCommand(address, self.Command.SET_MAIN_BATTERY_VOLTAGES, minimumBytes + maximumBytes)

    def setLogicVoltageLimits(self, address: int, minimumVoltage: float, maximumVoltage: float) -> bool:
        minimumBytes = _16BitValueToBytes(int(minimumVoltage * 10))
        maximumBytes = _16BitValueToBytes(int(maximumVoltage * 10))
        return self._sendSetCommand(address, self.Command.SET_LOGIC_BATTERY_VOLTAGES, minimumBytes + maximumBytes)

    def setM1DefaultAcceleration(self, address: int, acceleration: int) -> bool:
        accelerationBytes = _32BitValueToBytes(acceleration)
        return self._sendSetCommand(address, self.Command.SET_M1_DEFAULT_ACCELERATION_FOR_DUTY_CYCLE, accelerationBytes)

    def setM2DefaultAcceleration(self, address: int, acceleration: int) -> bool:
        accelerationBytes = _32BitValueToBytes(acceleration)
        return self._sendSetCommand(address, self.Command.SET_M2_DEFAULT_ACCELERATION_FOR_DUTY_CYCLE, accelerationBytes)

    def setM1DefaultVelocity(self, address: int, velocity: int) -> bool:
        velocityBytes = _16BitValueToBytes(velocity)
        return self._sendSetCommand(address, self.Command.SET_M1_DEFAULT_VELOCITY_FOR_POSITION, velocityBytes)

    def setM2DefaultVelocity(self, address: int, velocity: int) -> bool:
        velocityBytes = _16BitValueToBytes(velocity)
        return self._sendSetCommand(address, self.Command.SET_M2_DEFAULT_VELOCITY_FOR_POSITION, velocityBytes)

    def setS3S4S5Modes(self, address: int, s3Mode: S3Mode, s4Mode: S4Mode, s5Mode: S5Mode) -> bool:
        return self._sendSetCommand(address, self.Command.SET_S3S4S5_MODES, (s3Mode.value, s4Mode.value, s5Mode.value))

    def setDeadbandForRCAnalogControls(self, address: int, reversePercentage: float, forwardPercentage: float) -> bool:
        return self._sendSetCommand(address, self.Command.SET_RC_ANALOG_DEADBAND,
                                    (int(reversePercentage*10), int(forwardPercentage*10)))

    def setM1EncoderMode(self, address: int, encoderMode: EncoderMode):
        return self._sendSetCommand(address, self.Command.SET_M1_ENCODER_MODE, [encoderMode.getByte()])

    def setM2EncoderMode(self, address: int, encoderMode: EncoderMode):
        return self._sendSetCommand(address, self.Command.SET_M2_ENCODER_MODE, [encoderMode.getByte()])

    def setConfigurationSettings(self, address: int, settings: ConfigurationSettings):
        return None

    def ReadEncM1(self, address):
        return self._read4_1(address, self.Command.READ_M1_ENCODER_COUNT)

    def ReadEncM2(self, address):
        return self._read4_1(address, self.Command.READ_M2_ENCODER_COUNTS)

    def ReadSpeedM1(self, address):
        return self._read4_1(address, self.Command.READ_M1_ENCODER_COUNTS_PER_SECOND)

    def ReadSpeedM2(self, address):
        return self._read4_1(address, self.Command.READ_M2_ENCODER_COUNTS_PER_SECOND)

    def ResetEncoders(self, address):
        return self._write0(address, self.Command.RESET_QUADRATURE_ENCODERS)

    def ReadVersion(self, address):
        trys = self.__maxRetries
        while 1:
            self._port.flushInput()
            self._sendcommand(address, self.Command.READ_FIRMWARE_VERSION)
            str = ""
            passed = True
            for i in range(0, 48):
                data = self._port.read(1)
                if len(data):
                    val = ord(data)
                    self._updateCRC16SingleByte(val)
                    if (val == 0):
                        break
                    str += data[0]
                else:
                    passed = False
                    break
            if passed:
                crc = self._readchecksumword()
                if crc[0]:
                    if self.__crc16 & 0xFFFF == crc[1] & 0xFFFF:
                        return (1, str)
                    else:
                        time.sleep(0.01)
            trys -= 1
            if trys == 0:
                break
        return (0, 0)

    def SetEncM1(self, address, cnt):
        return self._write4(address, self.Command.SET_M1_QUADRATURE_ENCODER_REGISTER, cnt)

    def SetEncM2(self, address, cnt):
        return self._write4(address, self.Command.SET_M2_QUADRATURE_ENCODER_REGISTER, cnt)

    def ReadMainBatteryVoltage(self, address):
        return self._read2(address, self.Command.READ_MAIN_BATTERY_VOLTAGE)

    def ReadLogicBatteryVoltage(self, address, ):
        return self._read2(address, self.Command.READ_LOGIC_BATTERY_VOLTAGE)



    def SetM1VelocityPID(self, address, p, i, d, qpps):
        return self._write4444(address, self.Command.SET_M1_VELOCITY_PID_CONSTANTS, int(d * 65536), int(p * 65536), int(i * 65536), qpps)

    def SetM2VelocityPID(self, address, p, i, d, qpps):
        return self._write4444(address, self.Command.SET_M2_VELOCITY_PID_CONSTANTS, int(d * 65536), int(p * 65536), int(i * 65536), qpps)

    def ReadISpeedM1(self, address):
        return self._read4_1(address, self.Command.READ_M1_INSTANTANEOUS_COUNTS_PER_SECOND)

    def ReadISpeedM2(self, address):
        return self._read4_1(address, self.Command.READ_M2_INSTANTANEOUS_COUNTS_PER_SECOND)



    def ReadBuffers(self, address):
        val = self._read2(address, self.Command.READ_M1M2_BUFFER_LENGTH)
        if val[0]:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    def ReadPWMs(self, address):
        val = self._read4(address, self.Command.GETPWMS)
        if val[0]:
            pwm1 = val[1] >> 16
            pwm2 = val[1] & 0xFFFF
            if pwm1 & 0x8000:
                pwm1 -= 0x10000
            if pwm2 & 0x8000:
                pwm2 -= 0x10000
            return (1, pwm1, pwm2)
        return (0, 0, 0)

    def ReadCurrents(self, address):
        val = self._read4(address, self.Command.READ_M1M2_CURRENT_DRAW)
        if val[0]:
            cur1 = val[1] >> 16
            cur2 = val[1] & 0xFFFF
            if cur1 & 0x8000:
                cur1 -= 0x10000
            if cur2 & 0x8000:
                cur2 -= 0x10000
            return (1, cur1, cur2)
        return (0, 0, 0)

    def ReadM1VelocityPID(self, address):
        data = self._read_n(address, self.Command.READ_M1_VELOCITY_PID_COEFFICIENTS_AND_ACCELERATION_SETTINGS, 4)
        if data[0]:
            data[1] /= 65536.0
            data[2] /= 65536.0
            data[3] /= 65536.0
            return data
        return (0, 0, 0, 0, 0)

    def ReadM2VelocityPID(self, address):
        data = self._read_n(address, self.Command.READ_M2_VELOCITY_PID_COEFFICIENTS_AND_ACCELERATION_SETTINGS, 4)
        if data[0]:
            data[1] /= 65536.0
            data[2] /= 65536.0
            data[3] /= 65536.0
            return data
        return (0, 0, 0, 0, 0)


    def ReadMinMaxMainVoltages(self, address):
        val = self._read4(address, self.Command.READ_MAIN_BATTERY_SETTINGS)
        if val[0]:
            min = val[1] >> 16
            max = val[1] & 0xFFFF
            return (1, min, max)
        return (0, 0, 0)

    def ReadMinMaxLogicVoltages(self, address):
        val = self._read4(address, self.Command.READ_LOGIC_BATTERY_SETTINGS)
        if val[0]:
            min = val[1] >> 16
            max = val[1] & 0xFFFF
            return (1, min, max)
        return (0, 0, 0)

    def SetM1PositionPID(self, address, kp, ki, kd, kimax, deadzone, min, max):
        return self._write4444444(address, self.Command.SET_M1_POSITION_PID_COEFFICIENTS, int(kd * 1024), int(kp * 1024), int(ki * 1024),
                                  kimax, deadzone, min, max)

    def SetM2PositionPID(self, address, kp, ki, kd, kimax, deadzone, min, max):
        return self._write4444444(address, self.Command.SET_M2_POSITION_PID_COEFFICIENTS, int(kd * 1024), int(kp * 1024), int(ki * 1024),
                                  kimax, deadzone, min, max)

    def ReadM1PositionPID(self, address):
        data = self._read_n(address, self.Command.READ_M1_POSITION_PID_COEFFICIENTS, 7)
        if data[0]:
            data[1] /= 1024.0
            data[2] /= 1024.0
            data[3] /= 1024.0
            return data
        return (0, 0, 0, 0, 0, 0, 0, 0)

    def ReadM2PositionPID(self, address):
        data = self._read_n(address, self.Command.READ_M2_POSITION_PID_COEFFICIENTS, 7)
        if data[0]:
            data[1] /= 1024.0
            data[2] /= 1024.0
            data[3] /= 1024.0
            return data
        return (0, 0, 0, 0, 0, 0, 0, 0)



    def SetM1DefaultAccel(self, address, accel):
        return self._write4(address, self.Command.SET_M1_DEFAULT_ACCELERATION_FOR_DUTY_CYCLE, accel)

    def SetM2DefaultAccel(self, address, accel):
        return self._write4(address, self.Command.SET_M2_DEFAULT_ACCELERATION_FOR_DUTY_CYCLE, accel)

    def SetPinFunctions(self, address, S3mode, S4mode, S5mode):
        return self._write111(address, self.Command.SET_S3S4S5_MODES, S3mode, S4mode, S5mode)

    def ReadPinFunctions(self, address):
        trys = self.__maxRetries
        while 1:
            self._sendcommand(address, self.Command.READ_S3S4S5_MODES)
            val1 = self._readbyte()
            if val1[0]:
                val2 = self._readbyte()
                if val1[0]:
                    val3 = self._readbyte()
                    if val1[0]:
                        crc = self._readchecksumword()
                        if crc[0]:
                            if self.__crc16 & 0xFFFF != crc[1] & 0xFFFF:
                                return (0, 0)
                            return (1, val1[1], val2[1], val3[1])
            trys -= 1
            if trys == 0:
                break
        return (0, 0)

    def SetDeadBand(self, address, min, max):
        return self._write11(address, self.Command.SET_RC_ANALOG_DEADBAND, min, max)

    def GetDeadBand(self, address):
        val = self._read2(address, self.Command.READ_RC_ANALOG_DEADBAND)
        if val[0]:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    # Warning(TTL Serial): Baudrate will change if not already set to 38400.  Communications will be lost
    def RestoreDefaults(self, address):
        return self._write0(address, self.Command.RESTORE_FACTORY_SETTINGS)

    def ReadTemp(self, address):
        return self._read2(address, self.Command.READ_BOARD_TEMPERATURE)

    def ReadTemp2(self, address):
        return self._read2(address, self.Command.READ_SECOND_BOARD_TEMPERATURE)

    def ReadError(self, address):
        return self._read2(address, self.Command.READ_STATUS)

    def ReadEncoderModes(self, address):
        val = self._read2(address, self.Command.READ_M1M2_ENCODER_MODE)
        if val[0]:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    def SetM1EncoderMode(self, address, mode):
        return self._write1(address, self.Command.SET_M1_ENCODER_MODE, mode)

    def SetM2EncoderMode(self, address, mode):
        return self._write1(address, self.Command.SET_M2_ENCODER_MODE, mode)

    # saves active settings to NVM
    def WriteNVM(self, address):
        return self._write4(address, self.Command.WRITE_SETTINGS_TO_EEPROM, 0xE22EAB7A)

    # restores settings from NVM
    # Warning(TTL Serial): If baudrate changes or the control mode changes communications will be lost
    def ReadNVM(self, address):
        return self._write0(address, self.Command.READ_SETTINGS_FROM_EEPROM)

    # Warning(TTL Serial): If control mode is changed from packet serial mode when setting config communications will be lost!
    # Warning(TTL Serial): If baudrate of packet serial mode is changed communications will be lost!
    def SetConfig(self, address, config):
        return self._write2(address, self.Command.SETCONFIG, config)

    def GetConfig(self, address):
        return self._read2(address, self.Command.READ_STANDARD_CONFIGURATION_SETTINGS)

    def SetM1MaxCurrent(self, address, max):
        return self._write44(address, self.Command.SET_M1_CURRENT_LIMIT, max, 0)

    def SetM2MaxCurrent(self, address, max):
        return self._write44(address, self.Command.SET_M2_CURRENT_LIMIT, max, 0)

    def ReadM1MaxCurrent(self, address):
        data = self._read_n(address, self.Command.READ_M1_CURRENT_LIMIT, 2)
        if data[0]:
            return (1, data[1])
        return (0, 0)

    def ReadM2MaxCurrent(self, address):
        data = self._read_n(address, self.Command.READ_M2_CURRENT_LIMIT, 2)
        if data[0]:
            return (1, data[1])
        return (0, 0)

    def SetPWMMode(self, address, mode):
        return self._write1(address, self.Command.SET_PWM_MODE, mode)

    def ReadPWMMode(self, address):
        return self._read1(address, self.Command.GET_PWM_MODE)
