import time
from robot import Robot
from cv import Camera

# Initialise the camera and the robot
cam = Camera()
mirte = Robot()

mirte.setMotorSpeed("right", 60)
mirte.setMotorSpeed("left", 60)

while True:
    # Tells whether the stop line is in front of the robot
    stop_line = cam.stopLine()
    # If the stop line is in front of the robot, stop
    if stop_line:
        mirte.setMotorSpeed("right", 0)
        mirte.setMotorSpeed("left", 0)
    # Sleep for 1/30th of a second
    time.sleep(1 / 30)
