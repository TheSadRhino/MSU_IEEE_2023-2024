from abc import ABC, abstractmethod

import Action


class RunOnceAction(ABC, Action):
    def isFinished(self):
        return True

    def update(self):
        pass

    def onTermination(self):
        pass

    def onStart(self):
        self.runOnce()

    @abstractmethod
    def runOnce(self):
        pass
