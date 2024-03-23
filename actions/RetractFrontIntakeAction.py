from actions.RunOnceAction import RunOnceAction
from subsystems.Robot import Robot


class RetractFrontIntakeAction(RunOnceAction):

    def runOnce(self, robot: Robot = None):
        robot.raiseFrontIntake()


