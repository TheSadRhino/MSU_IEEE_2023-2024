import time

from actions.AtomicAction import AtomicAction
from actions.DeployIntakesAction import DeployIntakesAction
from actions.DriveWithGyroHeadingCorrection import DriveWithGyroHeadingCorrection
from actions.ParallelAction import ParallelAction
from actions.PowerButtonLED import PowerButtonLED
from actions.PrintButtonPressAction import PrintButtonPressAction
from actions.SeriesAction import SeriesAction
from actions.SetDrivetrainVelocity import SetDrivetrainVelocity
from actions.SetFrontIntakeVelocity import SetFrontIntakeVelocity
from actions.SetSideIntakeVelocity import SetSideIntakeVelocity
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
         SetDrivetrainVelocity(0, 0.25, 0),
         WaitForLeftSideDistanceLessThan(150),
         SetDrivetrainVelocity(0, 0, 0),
         ParallelAction(
             [DeployIntakesAction(),
              WaitAction(1)]
         ),
         ParallelAction(
             [SetSideIntakeVelocity(0.4),
              SetFrontIntakeVelocity(0.5)]
         ),
         AtomicAction(
             [DriveWithGyroHeadingCorrection(0.05, -0.25, 0),
              WaitAction(1)]
         ),
         ParallelAction(
             [SetSideIntakeVelocity(0),
              SetFrontIntakeVelocity(0)]
         )
         ]
    )
)





