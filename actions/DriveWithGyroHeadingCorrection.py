from actions.Action import Action
from subsystems.Robot import Robot
from subsystems.configurations import RobotConstants


class DriveWithGyroHeadingCorrection(Action):
    def __init__(self, xVelocity, yVelocity, desiredHeading):
        self.__robot = None
        self.__xVelocity = xVelocity
        self.__yVelocity = yVelocity
        self.__heading = desiredHeading

    def isFinished(self):
        return False

    def update(self):
        yaw, pitch, roll = self.__robot.getAngles()
        self.__robot.setNormalizedVelocity(self.__xVelocity, self.__yVelocity,
                                           (roll - self.__heading) * -RobotConstants.driveByGyroAngleErrorConstant)

    def onTermination(self):
        self.__robot.setNormalizedVelocity(0, 0, 0)

    def onStart(self, robot: Robot = None):
        self.__robot = robot
        yaw, pitch, roll = self.__robot.getAngles()
        self.__robot.setNormalizedVelocity(self.__xVelocity, self.__yVelocity,
                                           (roll - self.__heading) * -RobotConstants.driveByGyroAngleErrorConstant)
