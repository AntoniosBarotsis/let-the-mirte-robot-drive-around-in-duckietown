from mirte_msgs.msg import LineSegmentList
from line import LineSegment
import rospy

class ImageProcessor:
  def __init__(self):
    self.line_segments = []

    def lineSegmentCallback(data: LineSegmentList):
      self.line_segments = []
      for segment in data.segments:
        self.line_segments.append(LineSegment.from_message(segment))

    # Initialise node and subscribers
    rospy.init_node("image_processor", anonymous=True)
    rospy.Subscriber("line_segments", LineSegmentList, lineSegmentCallback)

    # Listen at 30 Hz
    self.rate = rospy.Rate(30)

  def sleep(self):
    self.rate.sleep()
  
  def getLines(self):
    return self.line_segments
