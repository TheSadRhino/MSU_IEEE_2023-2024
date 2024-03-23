from actions.Action import Action
from subsystems.Robot import Robot


class WaitForBackDistanceLessThan(Action):
    def __init__(self, distance):
        self.__robot = None
        self.__distance = distance

    def isFinished(self):
        return self.__distance > self.__robot.getRearRightTOFDistance()

    def update(self):
        pass

    def onTermination(self):
        pass

    def onStart(self, robot: Robot = None):
        self.__robot = robot
