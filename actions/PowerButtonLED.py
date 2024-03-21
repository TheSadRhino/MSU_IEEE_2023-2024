from actions.RunOnceAction import RunOnceAction
from subsystems.Robot import Robot


class PowerButtonLED(RunOnceAction):
    def __init__(self, ledOn):
        self.__ledOn = ledOn

    def runOnce(self, robot: Robot = None):
        robot.setButtonLED(self.__ledOn)
