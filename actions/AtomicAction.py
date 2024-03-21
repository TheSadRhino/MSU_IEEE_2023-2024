from actions.Action import Action
from subsystems.Robot import Robot
#Adjustment to push

class AtomicAction(Action):
    def __init__(self, actions: [Action]):
        self.__actions = actions

    def isFinished(self):
        for action in self.__actions:
            if action.isFinished():
                return True
        return False

    def onTermination(self):
        for action in self.__actions:
            action.onTermination()

    def update(self):
        for action in self.__actions:
            action.update()

    def onStart(self, robot: Robot = None):
        for action in self.__actions:
            action.onStart(robot)
