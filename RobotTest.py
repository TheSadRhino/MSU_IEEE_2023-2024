import asyncio
import multiprocessing
import threading
from concurrent.futures import ThreadPoolExecutor

from actions.AtomicAction import AtomicAction
from actions.ParallelAction import ParallelAction
from actions.PowerButtonLED import PowerButtonLED
from actions.PrintButtonPressAction import PrintButtonPressAction
from actions.SeriesAction import SeriesAction
from actions.WaitAction import WaitAction
from subsystems.Robot import Robot


#updateThread = threading.Thread(target=robot.updateRobot(), name="Robot Update Thread", daemon=True)
#updateThread.start()
robot = Robot()


def runActions():
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


runActions()



