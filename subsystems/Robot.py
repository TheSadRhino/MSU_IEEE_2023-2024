import board
from serial import Serial

from maestro.maestro import MaestroController
from roboclaw.bytebuffer import Roboclaw
from adafruit_as7341 import AS7341
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_vl53l0x import VL53L0X
from adafruit_vl6180x import VL6180X
from digitalio import DigitalInOut

from actions import Action
from subsystems.configurations import RobotConstants
from utilities import MathUtilities


class Robot:
    def __init__(self):
        self.__yawOffset = 0.0
        self.__pitchOffset = 0.0
        self.__rollOffset = 0.0

        self.__servoPositions = 6*[0]
        self.__servoPositions[RobotConstants.elevatorServoPin] = RobotConstants.elevatorServoDownPosition
        self.__servoPositions[RobotConstants.sideIntakeServoPin] = RobotConstants.sideIntakeServoUpPosition
        self.__servoPositions[RobotConstants.clawServoPin] = RobotConstants.clawServoRelaxPosition
        self.__servoPositions[RobotConstants.frontIntakeServoPin] = RobotConstants.frontIntakeServoUpPosition
        self.__servoPositions[RobotConstants.boosterAlignmentServoPin] = RobotConstants.boosterAlignmentServoUpPosition
        self.__servoPositions[RobotConstants.clawDeploymentServoPin] = RobotConstants.clawDeploymentServoUpPosition

        self.__roboclawSystem = Roboclaw(Serial(port=RobotConstants.motorControllerSerialPort,
                                                baudrate=RobotConstants.motorControllerBaudRate),
                                         address=RobotConstants.leftSideMotorControllerAddress)

        self.__maestroServoController = MaestroController(ttyStr=RobotConstants.servoControllerSerialPort,
                                                          baudRate=RobotConstants.servoControllerBaudRate)

        self.__i2c = board.I2C()

        self.__tofSensorPins = []
        self.__tofSensors = {}
        self.__tofDistances = {}

        for (address, pinGPIO, pinBoard, name) in RobotConstants.tofSensorPins:
            pin = DigitalInOut(pinGPIO)
            pin.switch_to_output(False)
            self.__tofSensorPins.append((pin, address, name))

        for pin, address, name in self.__tofSensorPins:
            pin.value = True

            if name == "VL53L0X":
                i2cDevice = VL53L0X(self.__i2c, address=address)
            else:
                i2cDevice = VL6180X(self.__i2c, address=address)
            self.__tofSensors.update((address, i2cDevice))
            pin.value = False

        self.__lightSensor = AS7341(self.__i2c, address=RobotConstants.lightSensorPins[0])
        self.__bno085 = BNO08X_I2C(self.__i2c, address=RobotConstants.bno085SensorPins[0])
        self.__bno085.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        self.__subsystemList = ()

        self._setupRobot()
        self._startTOFSensorRanging()

    def runAction(self, action: Action):
        action.onStart(self)

        while not action.isFinished():
            action.update()

        action.onTermination()

    def setYawOffset(self, angle):
        self.__yawOffset = angle

    def zeroAngles(self):
        self.__yawOffset = -self.__yaw
        self.__pitchOffset = -self.__pitch
        self.__rollOffset = -self.__roll

    def setElevatorServoPosition(self, position):
        self.__servoPositions[RobotConstants.elevatorServoPin] = position

    def raiseElevator(self):
        self.setElevatorServoPosition(RobotConstants.elevatorServoUpPosition)

    def lowerElevator(self):
        self.setElevatorServoPosition(RobotConstants.elevatorServoDownPosition)

    def setSideIntakeServoPosition(self, position):
        self.__servoPositions[RobotConstants.sideIntakeServoPin] = position

    def raiseSideIntake(self):
        self.setSideIntakeServoPosition(RobotConstants.sideIntakeServoUpPosition)

    def deploySideIntake(self):
        self.setSideIntakeServoPosition(RobotConstants.sideIntakeServoDownPosition)

    def setClawServoPosition(self, position):
        self.__servoPositions[RobotConstants.clawServoPin] = position

    def clampClaw(self):
        self.setClawServoPosition(RobotConstants.clawServoClampPosition)

    def relaxClaw(self):
        self.setClawServoPosition(RobotConstants.clawServoRelaxPosition)

    def openClaw(self):
        self.setClawServoPosition(RobotConstants.clawServoOpenPosition)

    def setFrontIntakeServoPosition(self, position):
        self.__servoPositions[RobotConstants.frontIntakeServoPin] = position

    def raiseFrontIntake(self):
        self.setFrontIntakeServoPosition(RobotConstants.frontIntakeServoUpPosition)

    def deployFrontIntake(self):
        self.setFrontIntakeServoPosition(RobotConstants.frontIntakeServoDownPosition)

    def setBoosterAlignmentServoPosition(self, position):
        self.__servoPositions[RobotConstants.boosterAlignmentServoPin] = position

    def deployBoosterAlignmentTool(self):
        self.setBoosterAlignmentServoPosition(RobotConstants.boosterAlignmentServoDownPosition)

    def retractBoosterAlignmentTool(self):
        self.setBoosterAlignmentServoPosition(RobotConstants.boosterAlignmentServoUpPosition)

    def setClawDeploymentServoPosition(self, position):
        self.__servoPositions[RobotConstants.clawDeploymentServoPin] = position

    def deployClaw(self):
        self.setClawDeploymentServoPosition(RobotConstants.clawDeploymentServoDownPosition)

    def retractClaw(self):
        self.setClawDeploymentServoPosition(RobotConstants.clawDeploymentServoUpPosition)

    def _setupRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.setupSystem()

    def _readInput(self):
        self.__quaternionI, self.__quaternionJ, self.__quaternionK, self.__quaternionReal = self.__bno085.quaternion

        self.__lightSensorChannels = self.__lightSensor.all_channels
        self.__415nm = self.__lightSensorChannels[0]
        self.__445nm = self.__lightSensorChannels[1]
        self.__480nm = self.__lightSensorChannels[2]
        self.__515nm = self.__lightSensorChannels[3]
        self.__555nm = self.__lightSensorChannels[4]
        self.__590nm = self.__lightSensorChannels[5]
        self.__630nm = self.__lightSensorChannels[6]
        self.__680nm = self.__lightSensorChannels[7]

        for address, device in self.__tofSensors:
            self.__tofDistances.update((address, device.range))

        self.__frontTOFDistance = self.__tofDistances.get(RobotConstants.frontTOFSensorPins[0])
        self.__leftFrontTOFDistance = self.__tofDistances.get(RobotConstants.frontLeftSideTOFSensorPins[0])
        self.__leftMiddleTOFDistance = self.__tofDistances.get(RobotConstants.middleLeftSideTOFSensorPins[0])
        self.__leftRearTOFDistance = self.__tofDistances.get(RobotConstants.rearLeftSideTOFSensorPins[0])
        self.__rearLeftTOFDistance = self.__tofDistances.get(RobotConstants.rearLeftRearTOFSensorPins[0])
        self.__rearRightTOFDistance = self.__tofDistances.get(RobotConstants.rearRightRearTOFSensorPins[0])
        self.__rightRearTOFDistance = self.__tofDistances.get(RobotConstants.rearRightSideTOFSensorPins[0])
        self.__rightFrontTOFDistance = self.__tofDistances.get(RobotConstants.frontRightSideTOFSensorPins[0])

        amperageSuccess, self.__frontIntakeAmperage, self.__rearIntakeAmperage = self.__roboclawSystem.read_currents(
            RobotConstants.intakeMotorControllerAddress)

    def _updateSystem(self):
        self.__yaw, self.__pitch, self.__roll = MathUtilities.quaternionToEuler(self.__quaternionI,
                                                                                self.__quaternionJ,
                                                                                self.__quaternionK,
                                                                                self.__quaternionReal)

        self.__yawFinal = self.__yaw + self.__yawOffset
        self.__pitchFinal = self.__pitch + self.__pitchOffset
        self.__rollFinal = self.__roll + self.__rollOffset

    def _writeOutput(self):
        for i in range(0, 6):
            self.__maestroServoController.setTarget(i, self.__servoPositions[i], True)

    def _updateRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.readInput()
        self._readInput()

        for subsystem in self.__subsystemList:
            subsystem.updateSystem()
        self._updateSystem()

        for subsystem in self.__subsystemList:
            subsystem.writeOutput()
        self._writeOutput()

    def _teardownRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.teardownSystem()

    def _startTOFSensorRanging(self):
        self._bootAllTOFSensors()
        for address, device in self.__tofSensors:
            device.start_continuous()

    def _bootAllTOFSensors(self):
        for pin, address, name in self.__tofSensorPins:
            pin.value = True

    def _shutdownAllTOFSensors(self):
        for pin, address, name in self.__tofSensorPins:
            pin.value = False
