from actions.RunOnceAction import RunOnceAction
from subsystems.Robot import Robot


class RetractFrontIntake(RunOnceAction):

    def runOnce(self, robot: Robot = None):
        robot.raiseFrontIntake()


