import Action


class SeriesAction(Action):
    def __init__(self, actions: [Action]):
        self.__actions = actions
        self.__currentAction = None

    def isFinished(self):
        return self.__currentAction is None and len(self.__actions) == 0

    def onTermination(self):
        pass

    def update(self):
        if self.__currentAction is None:
            if len(self.__actions) == 0:
                return

            self.__currentAction = self.__actions.pop(0)
            self.__currentAction.onStart()

        self.__currentAction.update()

        if self.__currentAction.isFinished():
            self.__currentAction.onTermination()
            self.__currentAction = None

    def onStart(self):
        pass
