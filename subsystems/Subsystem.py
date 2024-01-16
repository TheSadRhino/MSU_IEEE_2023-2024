from abc import ABC, abstractmethod


class Subsystem(ABC):
    @abstractmethod
    def setupSystem(self):
        pass

    @abstractmethod
    def readInput(self):
        pass

    @abstractmethod
    def updateSystem(self):
        pass

    @abstractmethod
    def writeOutput(self):
        pass

    @abstractmethod
    def teardownSystem(self):
        pass
