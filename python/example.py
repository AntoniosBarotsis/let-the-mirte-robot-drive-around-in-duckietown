from cv import ImageProcessor
processor = ImageProcessor()

while True:
  # Get the line segments
  line_segments = processor.getLines()
  # Print them
  for segment in line_segments:
    print(segment)
  print()
  # Sleep for 1/30th of a second
  processor.sleep()
