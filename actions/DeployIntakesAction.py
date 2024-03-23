from actions.RunOnceAction import RunOnceAction
from subsystems.Robot import Robot


class DeployIntakesAction(RunOnceAction):

    def runOnce(self, robot: Robot = None):
        robot.deployFrontIntake()
        robot.deploySideIntake()


