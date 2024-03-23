from subsystems.Robot import Robot

robot = Robot()
robot.zeroAngles()

while True:
    robot._readInput()
    yaw, pitch, roll = robot.getAngles()
    print("Yaw: " + yaw + " | Pitch: " + pitch + " | Roll: " + roll)
