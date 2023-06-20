import time
from mirte_duckietown import duckietown
from mirte_robot import robot

camera = duckietown.createCamera()
mirte = robot.createRobot()


while True:
    mirte.setMotorSpeed("left", 40)
    mirte.setMotorSpeed("right", 40)
    print(camera.getObstacles())
    if camera.seesObstacleOnLane():
        mirte.setMotorSpeed("left", 0)
        mirte.setMotorSpeed("right", 0)
        while True:
            if not camera.seesObstacleOnLane():
                break
            time.sleep(0.03)
    time.sleep(0.5)
