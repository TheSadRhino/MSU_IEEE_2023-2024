from actions.RunOnceAction import RunOnceAction
from subsystems.Robot import Robot


class RetractSideIntakeAction(RunOnceAction):

    def runOnce(self, robot: Robot = None):
        robot.raiseSideIntake()


