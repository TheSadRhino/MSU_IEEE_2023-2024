from abc import ABC, abstractmethod


class Action(ABC):
    @abstractmethod
    def isFinished(self) -> bool:
        return False

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def onStart(self):
        pass

    @abstractmethod
    def onTermination(self):
        pass
