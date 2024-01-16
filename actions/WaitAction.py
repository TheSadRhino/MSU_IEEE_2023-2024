import time
from math import pow

import Action
from subsystems.Robot import Robot


class WaitAction(Action):
    def __init__(self, timeToWait: float):
        self.__timeToWait = timeToWait
        self.__startTime = None

    def isFinished(self):
        return (time.time_ns() - self.__startTime) >= (self.__timeToWait * pow(10, 9))

    def update(self):
        pass

    def onTermination(self):
        pass

    def onStart(self, robot: Robot = None):
        self.__startTime = time.time_ns()
