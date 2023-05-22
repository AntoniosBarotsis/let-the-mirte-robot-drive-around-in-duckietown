from cv_py import colour, detect_line_type

colours = [colour.yellow, colour.white]

lines = detect_line_type(colours)

for line in lines:
    assert line.start.x != None
    assert line.start.y != None
    assert line.end.x != None
    assert line.end.y != None
