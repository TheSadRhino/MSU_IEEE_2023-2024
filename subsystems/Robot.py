import multiprocessing
import threading

import board
import busio
import digitalio
from adafruit_as7341 import AS7341
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C
from sensors.VL6180X import VL6180X
from serial import Serial
import RPi.GPIO as GPIO
from adafruit_extended_bus import ExtendedI2C

from actions import Action
from maestro.maestro import MaestroController
from roboclaw.bytebuffer import Roboclaw
from subsystems.configurations import RobotConstants
from utilities import MathUtilities, StatisticsUtility


class Robot:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)

        self.__ledOn = False
        self.__buttonPressed = False

        self.__yawOffset = 0.0
        self.__pitchOffset = 0.0
        self.__rollOffset = 0.0

        self.__normalizedXVelocity = 0.0
        self.__normalizedYVelocity = 0.0
        self.__normalizedHeadingVelocity = 0.0
        self.__normalizedWheelVelocities = 4*[0.0]
        self.__wheelVelocities = 4*[0]

        self.__normalizedIntakeVelocities = 2*[0.0]
        self.__intakeVelocities = 2*[0]
        self.__frontIntakeAmperageMovingAverageCalculator = StatisticsUtility \
            .MovingAverage(RobotConstants.frontIntakeCurrentMovingAverageWindow)
        self.__sideIntakeAmperageMovingAverageCalculator = StatisticsUtility \
            .MovingAverage(RobotConstants.sideIntakeCurrentMovingAverageWindow)
        self.__averageFrontIntakeAmperage = 0.0
        self.__averageSideIntakeAmperage = 0.0

        self.__desiredClimberPosition = 0
        self.__lastDesiredClimberPosition = 0
        self.__setClimberPosition = True
        self.__climberShouldOverrideLastMovement = True

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

        self.__i2cBus1 = busio.I2C(board.SCL, board.SDA)
        self.__i2cBus3 = ExtendedI2C(3)

        self.__tofSensorPins = []
        self.__tofSensors = {}
        self.__tofDistances = {}

        self.__frontTOFDistanceMovingAverageCalculator = StatisticsUtility\
            .MovingAverage(RobotConstants.tofSensorMovingAverageWindow)
        self.__leftFrontTOFDistanceMovingAverageCalculator = StatisticsUtility\
            .MovingAverage(RobotConstants.tofSensorMovingAverageWindow)
        self.__leftMiddleTOFDistanceMovingAverageCalculator = StatisticsUtility\
            .MovingAverage(RobotConstants.tofSensorMovingAverageWindow)
        self.__leftRearTOFDistanceMovingAverageCalculator = StatisticsUtility\
            .MovingAverage(RobotConstants.tofSensorMovingAverageWindow)
        self.__rearLeftTOFDistanceMovingAverageCalculator = StatisticsUtility\
            .MovingAverage(RobotConstants.tofSensorMovingAverageWindow)
        self.__rearRightTOFDistanceMovingAverageCalculator = StatisticsUtility\
            .MovingAverage(RobotConstants.tofSensorMovingAverageWindow)
        self.__rightRearTOFDistanceMovingAverageCalculator = StatisticsUtility\
            .MovingAverage(RobotConstants.tofSensorMovingAverageWindow)
        self.__rightFrontTOFDistanceMovingAverageCalculator = StatisticsUtility\
            .MovingAverage(RobotConstants.tofSensorMovingAverageWindow)

        for (address, pinGPIO, bus, name) in RobotConstants.tofSensorPins:
            GPIO.setup(pinGPIO, GPIO.OUT)
            GPIO.output(pinGPIO, False)
            self.__tofSensorPins.append((pinGPIO, address, bus, name))

        for pin, address, bus, name in self.__tofSensorPins:
            print(pin)
            if pin != 26 and pin != 19:
                GPIO.output(pin, True)

                if name == "VL53L0X":
                    print("We can't handle these, sorry")
                # if bus == 1:
                #     i2cDevice = VL53L0X(self.__i2cBus1)
                #     i2cDevice.set_address(address)
                # else:
                #     i2cDevice = VL53L0X(self.__i2cBus3)
                #     i2cDevice.set_address(address)
                else:
                    if bus == 1:
                        i2cDevice = VL6180X(self.__i2cBus1)
                        i2cDevice.set_address(address)
                    else:
                        i2cDevice = VL6180X(self.__i2cBus3)
                        i2cDevice.set_address(address)
                    print("success")
                    self.__tofSensors.update({address: i2cDevice})
                    self.__tofDistances.update({address: 255})

        self.__lightSensor = AS7341(self.__i2cBus3, address=RobotConstants.lightSensorPins[0])
        self.__lightSensorLEDCurrent = 0
        self.__lightSensorLEDOn = False

        self.__lightSensorStandardDeviationCalculators = RobotConstants\
            .lightSensorNumberOfChannels * [StatisticsUtility.MovingStandardDeviation(0)]

        for i in range(0, RobotConstants.lightSensorNumberOfChannels):
            self.__lightSensorStandardDeviationCalculators[i] = StatisticsUtility.MovingStandardDeviation(
                RobotConstants.lightSensorStandardDeviationWindow)

        self.__lightSensorStandardDeviations = RobotConstants.lightSensorNumberOfChannels * [0.0]

        self.__bno085 = BNO08X_I2C(self.__i2cBus3, address=RobotConstants.bno085SensorPins[0])
        self.__bno085.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        GPIO.setup(RobotConstants.redButtonPins[1], GPIO.IN)
        self.__buttonPressed = GPIO.input(RobotConstants.redButtonPins[1])
        GPIO.setup(RobotConstants.redLEDPins[1], GPIO.OUT)
        GPIO.output(RobotConstants.redLEDPins[1], self.__ledOn)

        self.__subsystemList = ()

        self._setupRobot()
        self._startTOFSensorRanging()

    def runAction(self, action: Action):
        self._updateRobotIteration()
        action.onStart(self)

        while not action.isFinished():
            action.update()
            self._updateRobotIteration()

        action.onTermination()
        self._updateRobotIteration()

    def setYawOffset(self, angle):
        self.__yawOffset = angle

    def zeroAngles(self):
        self.__yawOffset = -self.__yaw
        self.__pitchOffset = -self.__pitch
        self.__rollOffset = -self.__roll

    def getAngles(self):
        return self.__yawFinal, self.__pitchFinal, self.__rollFinal

    def getLightSensorChannels(self):
        return self.__lightSensorChannels

    def getLightSensorStandardDeviations(self):
        return self.__lightSensorStandardDeviations

    def setNormalizedVelocity(self, x, y, heading):
        self.__normalizedXVelocity = x
        self.__normalizedYVelocity = y
        self.__normalizedHeadingVelocity = heading

    def setFrontIntakeNormalizedVelocity(self, speed):
        self.__normalizedIntakeVelocities[0] = speed

    def setSideIntakeNormalizedVelocity(self, speed):
        self.__normalizedIntakeVelocities[1] = speed

    def setClimberMotorPosition(self, position):
        self.__desiredClimberPosition = position

    def setLightSensorLEDCurrent(self, current):
        self.__lightSensorLEDCurrent = current

    def setLightSensorLEDEnable(self, enableValue):
        self.__lightSensorLEDOn = enableValue

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

    def getRightFrontTOFDistance(self):
        return self.__rightFrontTOFDistance

    def getRightFrontTOFDistanceMovingAverage(self):
        return self.__rightFrontTOFDistanceMovingAverage

    def getFrontTOFDistance(self):
        return self.__frontTOFDistance

    def getFrontTOFDistanceMovingAverage(self):
        return self.__frontTOFDistanceMovingAverage

    def getLeftFrontTOFDistance(self):
        return self.__leftFrontTOFDistance

    def getLeftFrontTOFDistanceMovingAverage(self):
        return self.__leftFrontTOFDistanceMovingAverage

    def getLeftMiddleTOFDistance(self):
        return self.__leftMiddleTOFDistance

    def getLeftMiddleTOFDistanceMovingAverage(self):
        return self.__leftMiddleTOFDistanceMovingAverage

    def getLeftRearTOFDistance(self):
        return self.__leftRearTOFDistance

    def getLeftRearTOFDistanceMovingAverage(self):
        return self.__leftRearTOFDistanceMovingAverage

    def getRearLeftTOFDistance(self):
        return self.__rearLeftTOFDistance

    def getRearLeftTOFDistanceMovingAverage(self):
        return self.__rearLeftTOFDistanceMovingAverage

    def getRearRightTOFDistance(self):
        return self.__rearRightTOFDistance

    def getRearRightTOFDistanceMovingAverage(self):
        return self.__rearRightTOFDistanceMovingAverage

    def getRightRearTOFDistance(self):
        return self.__rightRearTOFDistance

    def getRightRearTOFDistanceMovingAverage(self):
        return self.__rightRearTOFDistanceMovingAverage

    def getButtonPress(self):
        return self.__buttonPressed

    def setButtonLED(self, ledOn):
        self.__ledOn = ledOn

    def _setupRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.setupSystem()

    def _readInput(self):
        self.__buttonPressed = GPIO.input(RobotConstants.redButtonPins[1])

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

        for address, device in self.__tofSensors.items():
            self.__tofDistances[address] = device.range

        #self.__frontTOFDistance = self.__tofDistances.get(RobotConstants.frontTOFSensorPins[0])
        self.__leftFrontTOFDistance = self.__tofDistances.get(RobotConstants.frontLeftSideTOFSensorPins[0])
        #self.__leftMiddleTOFDistance = self.__tofDistances.get(RobotConstants.middleLeftSideTOFSensorPins[0])
        #self.__leftRearTOFDistance = self.__tofDistances.get(RobotConstants.rearLeftSideTOFSensorPins[0])
        #self.__rearLeftTOFDistance = self.__tofDistances.get(RobotConstants.rearLeftRearTOFSensorPins[0])
        self.__rearRightTOFDistance = self.__tofDistances.get(RobotConstants.rearRightRearTOFSensorPins[0])
        #self.__rightRearTOFDistance = self.__tofDistances.get(RobotConstants.rearRightSideTOFSensorPins[0])
        #self.__rightFrontTOFDistance = self.__tofDistances.get(RobotConstants.frontRightSideTOFSensorPins[0])

        # amperageSuccess, self.__frontIntakeAmperage, self.__sideIntakeAmperage = self.__roboclawSystem.read_currents(
        #     RobotConstants.intakeMotorControllerAddress)

    def _updateSystem(self):
        if self.__desiredClimberPosition != self.__lastDesiredClimberPosition:
            self.__setClimberPosition = True

        self.__lastDesiredClimberPosition = self.__desiredClimberPosition

        self.__yaw, self.__pitch, self.__roll = MathUtilities.quaternionToEuler(self.__quaternionI,
                                                                                self.__quaternionJ,
                                                                                self.__quaternionK,
                                                                                self.__quaternionReal, True)

        self.__yawFinal = self.__yaw + self.__yawOffset
        self.__pitchFinal = self.__pitch + self.__pitchOffset
        self.__rollFinal = self.__roll + self.__rollOffset

        self.__normalizedWheelVelocities[0] = (self.__normalizedXVelocity - self.__normalizedYVelocity -
                                               self.__normalizedHeadingVelocity)
        self.__normalizedWheelVelocities[1] = (self.__normalizedXVelocity + self.__normalizedYVelocity -
                                               self.__normalizedHeadingVelocity)
        self.__normalizedWheelVelocities[2] = (self.__normalizedXVelocity - self.__normalizedYVelocity +
                                               self.__normalizedHeadingVelocity)
        self.__normalizedWheelVelocities[3] = (self.__normalizedXVelocity + self.__normalizedYVelocity +
                                               self.__normalizedHeadingVelocity)

        maximum = max((self.__normalizedWheelVelocities[0], self.__normalizedWheelVelocities[1],
                       self.__normalizedWheelVelocities[2], self.__normalizedWheelVelocities[3], 1.0))
        for i in range(0, len(self.__normalizedWheelVelocities)):
            self.__normalizedWheelVelocities[i] /= maximum
            self.__wheelVelocities[i] = int(self.__normalizedWheelVelocities[i] * RobotConstants
                                            .driveMotorMaximumVelocityTicksPerSecond)

        self.__intakeVelocities[0] = int(self.__normalizedIntakeVelocities[0] *
                                         RobotConstants.intakeMotorMaximumVelocityTicksPerSecond)
        self.__intakeVelocities[1] = int(self.__normalizedIntakeVelocities[1] *
                                         RobotConstants.intakeMotorMaximumVelocityTicksPerSecond)

        for i in range(0, len(self.__lightSensorStandardDeviationCalculators)):
            self.__lightSensorStandardDeviations[i] = self.__lightSensorStandardDeviationCalculators[i]\
                .addDataPointAndCalculateStandardDeviation(self.__lightSensorChannels[i])

        # self.__frontTOFDistanceMovingAverage = self.__frontTOFDistanceMovingAverageCalculator\
        #     .addDataPointAndCalculateAverage(self.__frontTOFDistance)
        self.__leftFrontTOFDistanceMovingAverage = self.__leftFrontTOFDistanceMovingAverageCalculator\
            .addDataPointAndCalculateAverage(self.__leftFrontTOFDistance)
        # self.__leftMiddleTOFDistanceMovingAverage = self.__leftMiddleTOFDistanceMovingAverageCalculator\
        #     .addDataPointAndCalculateAverage(self.__leftMiddleTOFDistance)
        # self.__leftRearTOFDistanceMovingAverage = self.__leftRearTOFDistanceMovingAverageCalculator\
        #     .addDataPointAndCalculateAverage(self.__leftRearTOFDistance)
        # self.__rearLeftTOFDistanceMovingAverage = self.__rearLeftTOFDistanceMovingAverageCalculator\
        #     .addDataPointAndCalculateAverage(self.__rearLeftTOFDistance)
        self.__rearRightTOFDistanceMovingAverage = self.__rearRightTOFDistanceMovingAverageCalculator\
            .addDataPointAndCalculateAverage(self.__rearRightTOFDistance)
        # self.__rightRearTOFDistanceMovingAverage = self.__rightRearTOFDistanceMovingAverageCalculator\
        #     .addDataPointAndCalculateAverage(self.__rightRearTOFDistance)
        # self.__rightFrontTOFDistanceMovingAverage = self.__rightFrontTOFDistanceMovingAverageCalculator\
        #     .addDataPointAndCalculateAverage(self.__rightFrontTOFDistance)

        # self.__averageFrontIntakeAmperage = self.__frontIntakeAmperageMovingAverageCalculator \
        #     .addDataAndCalculateAverage(self.__frontIntakeAmperage)
        # self.__averageSideIntakeAmperage = self.__sideIntakeAmperageMovingAverageCalculator \
        #     .addDataAndCalculateAverage(self.__sideIntakeAmperage)

    def _writeOutput(self):
        for i in range(0, 6):
            self.__maestroServoController.setTarget(i, self.__servoPositions[i], True)

        self.__roboclawSystem.speed_m1_m2(self.__wheelVelocities[0], self.__wheelVelocities[1],
                                          RobotConstants.leftSideMotorControllerAddress)
        self.__roboclawSystem.speed_m1_m2(self.__wheelVelocities[3], self.__wheelVelocities[2],
                                          RobotConstants.rightSideMotorControllerAddress)
        self.__roboclawSystem.speed_m1_m2(self.__intakeVelocities[0], self.__intakeVelocities[1],
                                          RobotConstants.intakeMotorControllerAddress)

        if self.__setClimberPosition:
            self.__roboclawSystem.speed_accel_deccel_position_m1(RobotConstants.climberMotorMaximumAcceleration,
                                                                 RobotConstants.climberMotorMaximumVelocity,
                                                                 RobotConstants.climberMotorMaximumDeceleration,
                                                                 self.__desiredClimberPosition,
                                                                 self.__climberShouldOverrideLastMovement,
                                                                 RobotConstants.climberMotorControllerAddress)
            self.__setClimberPosition = False

        GPIO.output(RobotConstants.redLEDPins[1], self.__ledOn)

        self.__lightSensor.led_current = self.__lightSensorLEDCurrent
        self.__lightSensor.led = self.__lightSensorLEDOn

    def updateRobot(self):
        while True:
            self._updateRobotIteration()

    def _updateRobotIteration(self):
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
        for address, device in self.__tofSensors.items():
            device.start_continuous()

    def _bootAllTOFSensors(self):
        for address, pin, bus, name in RobotConstants.functionalTOFSensorPins:
            GPIO.output(pin, True)

    def _shutdownAllTOFSensors(self):
        for address, pin, bus, name in RobotConstants.functionalTOFSensorPins:
            GPIO.output(pin, False)
