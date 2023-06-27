# pylint: skip-file

from mirte_robot import robot
from mirte_duckietown import duckietown

mirte = robot.createRobot()
camera = duckietown.createCamera(mirte)


while not (camera.seesStopLine()):
    camera.startFollowing()
camera.stopFollowing()
