# pylint: skip-file

from mirte_robot import robot_duckietown
from mirte_duckietown import duckietown

mirte = robot_duckietown.createRobot()
camera = duckietown.createCamera(mirte)


while not (camera.seesStopLine()):
    camera.startFollowing()
camera.stopFollowing()
