import time
from math import pow

import Action


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

    def onStart(self):
        self.__startTime = time.time_ns()
