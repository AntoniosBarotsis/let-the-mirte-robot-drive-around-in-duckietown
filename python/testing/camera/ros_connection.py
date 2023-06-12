import rospy
from sensor_msgs.msg import Image
from PIL import Image as PILImg
import numpy as np

from renderer import Renderer


class ImagePublisher:
    publisher = rospy.Publisher('/webcam/image_raw', Image, queue_size=10)

    def __init__(self):
        rospy.init_node("mocked_camera")

    def publish(self, image):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height = image.height
        msg.width = image.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * image.width
        msg.data = np.array(image).tobytes()
        self.publisher.publish(msg)


renderer = Renderer()
publisher = ImagePublisher()
img = renderer.render()
publisher.publish(img)
