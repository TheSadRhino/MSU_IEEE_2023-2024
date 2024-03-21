# --Sensors--
# Address, GPIO pin, physical pin location, sensor type
frontRightSideTOFSensorPins = (0x29, 20, 38, "VL53L0X")
frontTOFSensorPins = (0x30, 26, 37, "VL6180X")
frontLeftSideTOFSensorPins = (0x31, 13, 33, "VL6180X")
middleLeftSideTOFSensorPins = (0x32, 8, 24, "VL53L0X")
rearLeftSideTOFSensorPins = (0x33, 19, 35, "VL6180X")
rearLeftRearTOFSensorPins = (0x34, 6, 31, "VL53L0X")
rearRightRearTOFSensorPins = (0x35, 25, 22, "VL6180X")
rearRightSideTOFSensorPins = (0x36, 24, 18, "VL53L0X")

tofSensorPins = [frontRightSideTOFSensorPins, frontTOFSensorPins, frontLeftSideTOFSensorPins,
                 middleLeftSideTOFSensorPins, rearLeftSideTOFSensorPins, rearLeftRearTOFSensorPins,
                 rearRightRearTOFSensorPins, rearRightSideTOFSensorPins]

bno085SensorPins = (0x4A, 0, 0, "BNO085")
lightSensorPins = (0x39, 7, 26, "AS7341")

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
motorControllerSerialPort = "/dev/ttyAMA1"
motorControllerBaudRate = 115200

leftSideMotorControllerAddress = 0x80
rightSideMotorControllerAddress = 0x81
intakeMotorControllerAddress = 0x82
climberMotorControllerAddress = 0x83

frontIntakeCurrentThresholdForJams = 1.5
sideIntakeCurrentThresholdForJams = 1.5
driveMotorMaximumVelocityTicksPerSecond = 2250

climberMotorTuckedPosition = 0
climberMotorMountZiplinePosition = 500
climberMotorRideZiplinePosition = 750

