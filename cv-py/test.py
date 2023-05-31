from cv_py import colour, detect_line_type, detect_lane

colours = [colour.yellow, colour.white]

lines = detect_line_type(colours)

assert len(lines) != 0

for line in lines:
    assert line.start.x != None
    assert line.start.y != None
    assert line.end.x != None
    assert line.end.y != None

lanes = detect_lane(lines)

for line in lanes:
    assert line.start.x != None
    assert line.start.y != None
    assert line.end.x != None
    assert line.end.y != None

print("Passed")
