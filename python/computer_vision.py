from typing import List, Callable, Union
import rospy
from mirte_msgs.msg import LineSegmentList, Lane

class Listener:
  line_segment_callback: Union[Callable[[LineSegmentList], None], None]
  lane_callback: Union[Callable[[Lane], None], None]

  def __init__(self, line_segment_callback: Union[Callable[[LineSegmentList], None], None], lane_callback: Union[Callable[[Lane], None], None]) -> None:
    self.line_segment_callback = line_segment_callback
    self.lane_callback = lane_callback

  def listen(self):
    rospy.init_node("py_listener", anonymous=True)

    if self.line_segment_callback != None:
      rospy.Subscriber("line_segments", LineSegmentList, self.line_segment_callback)
    if self.lane_callback != None:
      rospy.Subscriber("lanes", Lane, self.lane_callback)

# ==== User Code ====

def line_segment_callback(data: LineSegmentList):
  print("line_segment_callback", len(data.segments))

def lane_callback(data: Lane):
  print("lane_callback", data.centre.origin.x)

listener = Listener(line_segment_callback, lane_callback)
listener.listen()

rospy.spin()
