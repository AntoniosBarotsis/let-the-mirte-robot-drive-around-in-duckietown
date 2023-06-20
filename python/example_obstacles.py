import time
from mirte_duckietown import duckietown

camera = duckietown.createCamera()

while True:
    print(camera.getObstacles())
    time.sleep(0.03)
