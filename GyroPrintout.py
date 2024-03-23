from subsystems.Robot import Robot

robot = Robot()
robot._readInput()
robot._updateSystem()

robot.zeroAngles()

while True:
    robot._readInput()
    robot._updateSystem()

    yaw, pitch, roll = robot.getAngles()
    print(f"Yaw: %.2f | Pitch %.2f | Roll %.2f", (yaw, pitch, roll))
