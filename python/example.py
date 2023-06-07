from cv import ImageProcessor
proccessor = ImageProcessor()

while True:
  # Get the line segments
  line_segments = proccessor.getLines()
  # Print them
  for segment in line_segments:
    print(segment)
  print()
  # Sleep for 1/30th of a second
  proccessor.sleep()
