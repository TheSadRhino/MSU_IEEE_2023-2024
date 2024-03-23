from actions.Action import Action
from subsystems.Robot import Robot


class WaitForButtonPress(Action):
    def __init__(self):
        self.__robot = None

    def isFinished(self):
        return self.__robot.getButtonPress()

    def update(self):
        pass

    def onTermination(self):
        pass

    def onStart(self, robot: Robot = None):
        self.__robot = robot
