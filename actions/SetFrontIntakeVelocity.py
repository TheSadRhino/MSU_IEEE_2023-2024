from actions.RunOnceAction import RunOnceAction
from subsystems.Robot import Robot


class SetFrontIntakeVelocity(RunOnceAction):

    def __init__(self, velocity):
        self.__velocity = velocity

    def runOnce(self, robot: Robot = None):
        robot.setFrontIntakeNormalizedVelocity(self.__velocity)
