# pylint: skip-file
from mirte_robot import robot
mirte=robot.createRobot()
from duckietown import Camera
camera = Camera()
import time


mirte.setMotorSpeed('left', 60)
mirte.setMotorSpeed('right', 60)
while True:
  if camera.stopLine():
    mirte.setMotorSpeed('left', 0)
    mirte.setMotorSpeed('right', 0)
  time.sleep(0.03)

