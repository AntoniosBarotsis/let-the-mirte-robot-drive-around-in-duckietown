import time
from mirte_robot import robot_duckietown
from mirte_duckietown import duckietown

mirte = robot_duckietown.createRobot()
camera = duckietown.createCamera()

mirte.setMotorSpeed("left", 60)
mirte.setMotorSpeed("right", 60)
while True:
    if camera.seesStopLine():
        mirte.setMotorSpeed("left", 0)
        mirte.setMotorSpeed("right", 0)
        break
    time.sleep(0.03)
