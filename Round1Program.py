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
         SetDrivetrainVelocity(0, 0.175, 0),
         WaitForLeftSideDistanceLessThan(150),
         SetDrivetrainVelocity(0, 0, 0),
         ParallelAction(
             [DeployIntakesAction(),
              SetSideIntakeVelocity(0.4),
              SetFrontIntakeVelocity(0.5)]
         ),
         SetDrivetrainVelocity(0.025, -0.2, 0),
         WaitAction(0.4),
         ParallelAction(
             [SetDrivetrainVelocity(-0.05, 0.15, 0),
              WaitForLeftSideDistanceLessThan(150)]
         ),
         ParallelAction(
             [RetractSideIntakeAction(),
              SetSideIntakeVelocity(0),
              WaitAction(0.5)]
         ),
         SetDrivetrainVelocity(-0.05, -0.05, 0),
         SetDrivetrainVelocity(0.175, 0, 0),
         WaitAction(0.75),
         SetDrivetrainVelocity(-0.175, 0, 0),
         WaitAction(0.75),
         SetDrivetrainVelocity(0, -0.065, 0),
         WaitAction(0.25),
         SetDrivetrainVelocity(0.35, 0, 0),
         ParallelAction(
             [WaitAction(3.5),
              RetractFrontIntakeAction(),
              SetFrontIntakeVelocity(0)]
         ),
         SetDrivetrainVelocity(0, 0, 0),
         TurnForAngle(90, 0.1)
         ]
    )
)





