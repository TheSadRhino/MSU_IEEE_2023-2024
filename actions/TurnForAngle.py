import math

from actions.Action import Action
from subsystems.Robot import Robot
from subsystems.configurations import RobotConstants


class TurnForAngle(Action):
    def __init__(self, angle, speed):
        self.__initialYaw = None
        self.__initialPitch = None
        self.__initialRoll = None
        self.__robot = None
        self.__angle = angle
        self.__speed = speed

    def isFinished(self):
        yaw, pitch, roll = self.__robot.getAngles()

        return roll > self.__initialRoll + self.__angle

    def update(self):
        pass

    def onTermination(self):
        self.__robot.setNormalizedVelocity(0, 0, 0)

    def onStart(self, robot: Robot = None):
        self.__robot = robot
        self.__initialYaw, self.__initialPitch, self.__initialRoll = self.__robot.getAngles()

        self.__robot.setNormalizedVelocity(0, 0, math.copysign(self.__speed, self.__angle))
