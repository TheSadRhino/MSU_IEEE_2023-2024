from abc import ABC, abstractmethod

from actions.Action import Action
from subsystems.Robot import Robot


class RunOnceAction(Action, ABC):
    def isFinished(self):
        return True

    def update(self):
        pass

    def onTermination(self):
        pass

    def onStart(self, robot: Robot = None):
        self.runOnce(robot)

    @abstractmethod
    def runOnce(self, robot: Robot = None):
        pass
