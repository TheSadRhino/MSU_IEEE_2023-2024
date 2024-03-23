from actions.RunOnceAction import RunOnceAction
from subsystems.Robot import Robot


class SetSideIntakeVelocity(RunOnceAction):
    def __init__(self, velocity):
        self.__velocity = velocity

    def runOnce(self, robot: Robot = None):
        robot.setSideIntakeNormalizedVelocity(self.__velocity)
