import multiprocessing
import threading

from actions.AtomicAction import AtomicAction
from actions.ParallelAction import ParallelAction
from actions.PowerButtonLED import PowerButtonLED
from actions.PrintButtonPressAction import PrintButtonPressAction
from actions.SeriesAction import SeriesAction
from actions.WaitAction import WaitAction
from subsystems.Robot import Robot

robot = Robot()

updateThread = threading.Thread(target=robot.updateRobot(), name="Robot Update Thread", daemon=True)
updateThread.start()

print("post initialize")

robot.runAction(
    SeriesAction(
        [AtomicAction(
            [WaitAction(15),
             PrintButtonPressAction()]
         ),
         ParallelAction(
            [WaitAction(5), PowerButtonLED(True)]
         )]
    )
)

robot.runAction(PowerButtonLED(False))
