import time

from maestro.maestro import MaestroController
from subsystems.configurations import RobotConstants

maestroServoController = MaestroController(ttyStr=RobotConstants.servoControllerSerialPort,
                                           baudRate=RobotConstants.servoControllerBaudRate)

position = RobotConstants.clawDeploymentServoUpPosition
maestroServoController.setTarget(5, position, False)
time.sleep(3)

position = RobotConstants.clawDeploymentServoDownPosition
maestroServoController.setTarget(5, position, False)
time.sleep(3)