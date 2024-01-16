from actions import Action


class Robot:
    def __init__(self):

        self.__subsystemList = ()

        self._setupRobot()

    def runAction(self, action: Action):
        action.onStart(self)

        while not action.isFinished():
            action.update()

        action.onTermination()

    def _setupRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.setupSystem()

    def _updateRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.readInput()

        for subsystem in self.__subsystemList:
            subsystem.updateSystem()

        for subsystem in self.__subsystemList:
            subsystem.writeOutput()

    def _teardownRobot(self):
        for subsystem in self.__subsystemList:
            subsystem.teardownSystem()
