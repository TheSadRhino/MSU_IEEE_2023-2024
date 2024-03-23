from actions.AtomicAction import AtomicAction
from actions.DeployIntakesAction import DeployIntakesAction
from actions.DriveWithGyroHeadingCorrection import DriveWithGyroHeadingCorrection
from actions.ParallelAction import ParallelAction
from actions.PowerButtonLED import PowerButtonLED
from actions.PrintButtonPressAction import PrintButtonPressAction
from actions.SeriesAction import SeriesAction
from actions.SetFrontIntakeVelocity import SetFrontIntakeVelocity
from actions.SetSideIntakeVelocity import SetSideIntakeVelocity
from actions.WaitAction import WaitAction
from actions.WaitForButtonPress import WaitForButtonPress
from actions.ZeroGyroValues import ZeroGyroValues
from subsystems.Robot import Robot

robot = Robot()

robot.runAction(
    SeriesAction(
        [WaitForButtonPress(),
         ParallelAction(
             [ZeroGyroValues(),
              DeployIntakesAction(),
              WaitAction(1)]
         ),
         SetSideIntakeVelocity(0.2),
         SetFrontIntakeVelocity(0.2),
         AtomicAction(
             [DriveWithGyroHeadingCorrection(0.05, -0.1, 0),
              WaitAction(2)]
         ),
         SetSideIntakeVelocity(0),
         SetFrontIntakeVelocity(0),
         ]
    )
)





