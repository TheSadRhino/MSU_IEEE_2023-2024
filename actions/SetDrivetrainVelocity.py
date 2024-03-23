from actions.RunOnceAction import RunOnceAction
from subsystems.Robot import Robot


class SetDrivetrainVelocity(RunOnceAction):
    def __init__(self, xVelocity, yVelocity, headingVelocity):
        self.__xVelocity = xVelocity
        self.__yVelocity = yVelocity
        self.__headingVelocity = headingVelocity

    def runOnce(self, robot: Robot = None):
        robot.setNormalizedVelocity(self.__xVelocity, self.__yVelocity, self.__headingVelocity)
        print(robot.getAngles()[0])
