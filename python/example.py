import time
from mirte_robot import robot
from mirte_duckietown import duckietown

mirte = robot.createRobot()
camera = duckietown.createCamera()

mirte.setMotorSpeed("left", 60)
mirte.setMotorSpeed("right", 60)
while True:
    if camera.seesStopLine():
        mirte.setMotorSpeed("left", 0)
        mirte.setMotorSpeed("right", 0)
    time.sleep(0.03)
