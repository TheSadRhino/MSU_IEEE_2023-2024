from digitalio import DigitalInOut

from actions import Action
from configurations.RobotConstants import tofSensorPins


class Robot:
    def __init__(self):
        self.__tofSensorPins = []

        for (address, pin) in tofSensorPins:
            self.__tofSensorPins.append(DigitalInOut(pin))

        self.__tofSensorList = []


        self.__subsystemList = ()

        self._setupRobot()

    def runAction(self, action: Action):
        action.onStart(self)

        while not action.isFinished():
            action.update()

        action.onTermination()

    def _setupRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.setupSystem()

    def _updateRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.readInput()

        for subsystem in self.__subsystemList:
            subsystem.updateSystem()

        for subsystem in self.__subsystemList:
            subsystem.writeOutput()

    def _teardownRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.teardownSystem()
