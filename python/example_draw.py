# pylint: skip-file

from mirte_duckietown import duckietown
from mirte_duckietown._util import intersectWithHorizontalLine
import cv2

# Initialise the camera
processor = duckietown.createCamera()

# Example from https://stackoverflow.com/questions/45322630/how-to-detect-lines-in-opencv
while True:
    img = processor.getImage()
    if img is None:
        print("No image")
        continue

    width = img.shape[1]
    height = img.shape[0]

    # for line in processor.getLines():
    # if line.colour == Colour.YELLOW:
    #     colour = (255, 255, 0)
    # elif line.colour == Colour.WHITE:
    #     colour = (255, 255, 255)
    # if line.colour == Colour.RED:
    #     colour = (255, 189, 15)
    #     cv2.line(img, (int(width*line.start.x_coord), int(height*line.start.y_coord)), (int(width*line.end.x_coord), int(height*line.end.y_coord)), colour, 5)

    lane = processor.getLane()
    l = lane.left_line
    c = lane.centre_line
    r = lane.right_line

    stop_line = processor.getStopLine()

    lines = [
        (l, (0, 0, 255)),
        (c, (0, 255, 0)),
        (r, (255, 0, 0)),
        (stop_line, (64, 255, 255)),
    ]

    for (line, colour) in lines:
        if line is not None:
            start = intersectWithHorizontalLine(line, 1)
            end = intersectWithHorizontalLine(line, 0.58)
            if start is not None and end is not None:
                cv2.line(
                    img,
                    (int(width * start), int(height)),
                    (int(width * end), int(height * 0.58)),
                    colour,
                    5,
                )

    im_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imshow("lines", im_rgb)
    cv2.waitKey(1)
