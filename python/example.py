from cv import Camera

# Initialise the camera
processor = Camera()

while True:
    # Get the stop line
    stop_line = processor.getStopLine()
    # Print the stop line
    print(f"Stop line: {stop_line}")
    print()
    # Sleep for 1/30th of a second
    processor.sleep()
