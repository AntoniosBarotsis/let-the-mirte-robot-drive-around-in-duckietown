from cv_py import colour, detect_line_type, detect_lane

colours = [colour.yellow, colour.white]

lines = detect_line_type(colours)

assert len(lines) != 0

for line in lines:
    assert line.start.x != None
    assert line.start.y != None
    assert line.end.x != None
    assert line.end.y != None

lane = detect_lane(lines)

if lane != None:
    assert lane.start.x != None
    assert lane.start.y != None
    assert lane.end.x != None
    assert lane.end.y != None

    print("Passed")
else:
    print("lane was None")
