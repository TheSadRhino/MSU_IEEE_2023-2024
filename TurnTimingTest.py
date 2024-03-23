import time

from actions.AtomicAction import AtomicAction
from actions.DeployIntakesAction import DeployIntakesAction
from actions.DriveUntilPitchGreaterThan import DriveUntilPitchGreaterThan
from actions.DriveWithGyroHeadingCorrection import DriveWithGyroHeadingCorrection
from actions.ParallelAction import ParallelAction
from actions.PowerButtonLED import PowerButtonLED
from actions.PrintButtonPressAction import PrintButtonPressAction
from actions.RetractFrontIntakeAction import RetractFrontIntakeAction
from actions.RetractSideIntakeAction import RetractSideIntakeAction
from actions.SeriesAction import SeriesAction
from actions.SetDrivetrainVelocity import SetDrivetrainVelocity
from actions.SetFrontIntakeVelocity import SetFrontIntakeVelocity
from actions.SetSideIntakeVelocity import SetSideIntakeVelocity
from actions.TurnForAngle import TurnForAngle
from actions.TurnForNegativeAngle import TurnForNegativeAngle
from actions.WaitAction import WaitAction
from actions.WaitForButtonPress import WaitForButtonPress
from actions.WaitForLeftSideDistanceLessThan import WaitForLeftSideDistanceLessThan
from actions.ZeroGyroValues import ZeroGyroValues
from subsystems.Robot import Robot

time.sleep(15)

robot = Robot()


robot.runAction(
    SeriesAction(
        [WaitForButtonPress(),
         ZeroGyroValues(),
         SetDrivetrainVelocity(0, 0, 0.075),
         WaitAction(0.75),
         SetDrivetrainVelocity(0, 0, 0)
         ]
    ))





