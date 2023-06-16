import time
from test import Follower
from cv import Camera


# Initialise the camera
cam = Camera()
mirte = Follower()

mirte.set_motor_speed("right", 60)
mirte.set_motor_speed("left", 60)


while True:
    # Get the stop line
    stop_line = cam.stopLineDist()
    # Print the stop line
    print(f"Stop line: {stop_line}")
    print()
    if stop_line is not None and stop_line < 0.4:
        mirte.set_motor_speed("right", 0)
        mirte.set_motor_speed("left", 0)
    # Sleep for 1/30th of a second
    time.sleep(1 / 30)
