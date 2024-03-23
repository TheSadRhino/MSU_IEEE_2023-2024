from actions.Action import Action
from subsystems.Robot import Robot
from subsystems.configurations import RobotConstants


class DriveUntilPitchGreaterThan(Action):
    def __init__(self, xVelocity, yVelocity, pitch):
        self.__robot = None
        self.__xVelocity = xVelocity
        self.__yVelocity = yVelocity
        self.__pitch = pitch

    def isFinished(self):
        yaw, pitch, roll = self.__robot.getAngles()

        return pitch > self.__pitch

    def update(self):
        pass

    def onTermination(self):
        self.__robot.setNormalizedVelocity(0, 0, 0)

    def onStart(self, robot: Robot = None):
        self.__robot = robot
        self.__robot.setNormalizedVelocity(self.__xVelocity, self.__yVelocity, 0)
