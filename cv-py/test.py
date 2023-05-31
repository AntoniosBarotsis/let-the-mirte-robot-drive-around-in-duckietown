from cv_py import Colour, detect_line_type, detect_lane

colours = [Colour.yellow, Colour.white]

lines = detect_line_type(colours)

assert len(lines) != 0

for line in lines:
    assert line.start.x != None
    assert line.start.y != None
    assert line.end.x != None
    assert line.end.y != None

lane = detect_lane(lines)

assert lane != None

assert lane.centre != None
assert lane.left != None
assert lane.right != None

assert lane.centre.origin != None
assert lane.centre.direction != None
assert lane.left.origin != None
assert lane.left.direction != None
assert lane.right.origin != None
assert lane.right.direction != None

print("Passed")
