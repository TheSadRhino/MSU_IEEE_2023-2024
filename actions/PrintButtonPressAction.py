from actions.Action import Action
from subsystems.Robot import Robot


class PrintButtonPressAction(Action):
    def __init__(self):
        self.__robot = None

    def isFinished(self):
        return False

    def update(self):
        print(self.__robot.getButtonPress())

    def onTermination(self):
        pass

    def onStart(self, robot: Robot = None):
        self.__robot = robot
