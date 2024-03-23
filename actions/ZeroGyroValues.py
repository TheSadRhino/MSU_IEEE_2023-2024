from actions.RunOnceAction import RunOnceAction
from subsystems.Robot import Robot


class ZeroGyroValues(RunOnceAction):

    def runOnce(self, robot: Robot = None):
        robot.zeroAngles()
