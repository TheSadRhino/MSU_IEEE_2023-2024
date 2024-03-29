from actions.Action import Action
from subsystems.Robot import Robot


class ParallelAction(Action):
    def __init__(self, actions: [Action]):
        self.__actions = actions

    def isFinished(self):
        for action in self.__actions:
            if not action.isFinished():
                return False
        return True

    def onTermination(self):
        for action in self.__actions:
            action.onTermination()

    def update(self):
        for action in self.__actions:
            action.update()

    def onStart(self, robot: Robot = None):
        for action in self.__actions:
            action.onStart(robot)
