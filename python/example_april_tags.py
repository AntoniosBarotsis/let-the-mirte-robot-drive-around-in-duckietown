import os
import time
from mirte_duckietown import duckietown

camera = duckietown.createCamera()

while True:
    tags = camera.getAprilTags()
    print("Tags length: ", len(tags))
    for tag in tags:
        print("tag_id: ", tag.tag_id, "sign: ", tag.toSign())
    time.sleep(0.2)
    os.system("clear")
