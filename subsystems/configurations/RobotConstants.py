# --Sensors--
# Address, GPIO pin, I2C Bus, sensor type
frontRightSideTOFSensorPins = (0x29, 20, 1, "VL53L0X")
frontTOFSensorPins = (0x30, 26, 1, "VL6180X")
frontLeftSideTOFSensorPins = (0x31, 13, 1, "VL6180X")
middleLeftSideTOFSensorPins = (0x32, 8, 1, "VL53L0X")
rearLeftSideTOFSensorPins = (0x33, 19, 1, "VL6180X")
rearLeftRearTOFSensorPins = (0x34, 6, 1, "VL53L0X")
rearRightRearTOFSensorPins = (0x35, 25, 3, "VL6180X")
rearRightSideTOFSensorPins = (0x36, 9, 3, "VL53L0X")

tofSensorPins = [frontRightSideTOFSensorPins, frontTOFSensorPins, frontLeftSideTOFSensorPins,
                 middleLeftSideTOFSensorPins, rearLeftSideTOFSensorPins, rearLeftRearTOFSensorPins,
                 rearRightRearTOFSensorPins, rearRightSideTOFSensorPins]
functionalTOFSensorPins = [rearRightRearTOFSensorPins, frontLeftSideTOFSensorPins]

tofSensorMovingAverageWindow = 3

bno085SensorPins = (0x4A, 0, 3, "BNO085")
lightSensorPins = (0x39, 7, 3, "AS7341")
lightSensorStandardDeviationWindow = 15
lightSensorNumberOfChannels = 8

redButtonPins = (None, 17, 11, "Limit Switch")
redLEDPins = (None, 27, 13, "LED")

# --Servos--
servoControllerSerialPort = "/dev/ttyAMA0"
servoControllerBaudRate = 115200

elevatorServoPin = 0
elevatorServoUpPosition = 1700
elevatorServoDownPosition = 1150

sideIntakeServoPin = 1
sideIntakeServoUpPosition = 2150
sideIntakeServoDownPosition = 1350

clawServoPin = 2
clawServoClampPosition = 1250
clawServoRelaxPosition = 1300
clawServoOpenPosition = 1350

frontIntakeServoPin = 3
frontIntakeServoUpPosition = 935
frontIntakeServoDownPosition = 2135

boosterAlignmentServoPin = 4
boosterAlignmentServoUpPosition = 2055
boosterAlignmentServoDownPosition = 825

clawDeploymentServoPin = 5
clawDeploymentServoUpPosition = 2175
clawDeploymentServoDownPosition = 790

# --Motors--
motorControllerSerialPort = "/dev/ttyAMA3"
motorControllerBaudRate = 115200

leftSideMotorControllerAddress = 0x80
rightSideMotorControllerAddress = 0x81
intakeMotorControllerAddress = 0x82
climberMotorControllerAddress = 0x83

frontIntakeCurrentThresholdForJams = 1.5
frontIntakeCurrentMovingAverageWindow = 5
sideIntakeCurrentThresholdForJams = 1.5
sideIntakeCurrentMovingAverageWindow = 5
driveMotorMaximumVelocityTicksPerSecond = 2250

climberMotorTuckedPosition = 0
climberMotorMountZiplinePosition = 500
climberMotorRideZiplinePosition = 750

climberMotorMaximumAcceleration = 111000
climberMotorMaximumDeceleration = 20000
climberMotorMaximumVelocity = 500

