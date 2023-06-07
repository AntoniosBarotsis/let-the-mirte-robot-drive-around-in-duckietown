import rospy
from sensor_msgs.msg import Image

class Camera:
    current_image: Image = None

    def __init__(self):
        rospy.init_node("camera_py", anonymous=True)
        rospy.Subscriber("/webcam/image_raw", Image, self.ros_callback)

    def ros_callback(self, data: Image):
        self.current_image = data

    def get_ros_image(self):
        return self.current_image

    def get_opencv_image(self):
        print("todo")

