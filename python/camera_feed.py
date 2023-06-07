import rospy
from sensor_msgs.msg import Image
import cv2  # Import needed since otherwise CvBridge crashes
from cv_bridge import CvBridge

class Camera:
    current_image: Image = None
    bridge = CvBridge()

    def __init__(self):
        rospy.init_node("camera_py", anonymous=True)
        rospy.Subscriber("/webcam/image_raw", Image, self.ros_callback)

    def ros_callback(self, data: Image):
        self.current_image = data

    def get_ros_image(self):
        return self.current_image

    def get_opencv_image(self):
        return self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding="passthrough")

