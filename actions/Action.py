from abc import ABC, abstractmethod

from subsystems import Robot


class Action(ABC):
    @abstractmethod
    def isFinished(self) -> bool:
        return False

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def onStart(self, robot: Robot = None):
        pass

    @abstractmethod
    def onTermination(self):
        pass
