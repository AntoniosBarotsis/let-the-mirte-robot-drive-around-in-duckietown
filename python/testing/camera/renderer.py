import pyrender.material
import trimesh.visual
import numpy as np
import transformations as tf
from PIL import Image as PILImage

import rospy
from sensor_msgs.msg import Image

class Renderer:
    __camera_node = None
    __camera_pose = None
    __scene = None

    def __init__(self, scenario='duckievoort'):
        # Initialize camera pose
        self.__camera_pose = tf.identity_matrix()
        rotation_matrix = tf.rotation_matrix(np.radians(90), [1, 0, 0])  # Parallel to the ground
        self.__camera_pose = np.matmul(self.__camera_pose, rotation_matrix)

        # Create materials
        mat_black = pyrender.MetallicRoughnessMaterial(
            metallicFactor=0.0,
            alphaMode='OPAQUE',
            baseColorFactor=(0.0, 0.0, 0.0, 1.0)
        )
        mat_white = pyrender.MetallicRoughnessMaterial(
            metallicFactor=0.0,
            alphaMode='OPAQUE',
            baseColorFactor=(1.0, 1.0, 1.0, 1.0)
        )
        mat_yellow = pyrender.MetallicRoughnessMaterial(
            metallicFactor=0.0,
            alphaMode='OPAQUE',
            baseColorFactor=(1.0, 1.0, 0.0, 1.0)
        )

        # Load meshes
        if scenario == 'duckievoort':
            mesh_black_raw = trimesh.load('./testing/towns/duckievoort/black.obj')
            mesh_white_raw = trimesh.load('./testing/towns/duckievoort/white.obj')
            mesh_yellow_raw = trimesh.load('./testing/towns/duckievoort/yellow.obj')
            self.position(20.5, 0.4, -35)
            self.rotate(-18, 'y')
        else:
            raise Exception('Invalid scenario')

        # Convert & add materials
        mesh_black = pyrender.Mesh.from_trimesh(mesh_black_raw, material=mat_black)
        mesh_white = pyrender.Mesh.from_trimesh(mesh_white_raw, material=mat_white)
        mesh_yellow = pyrender.Mesh.from_trimesh(mesh_yellow_raw, material=mat_yellow)

        # Create poses
        pose_track = tf.identity_matrix()
        rotation_matrix = tf.rotation_matrix(np.radians(90), [1, 0, 0])
        pose_track = np.matmul(pose_track, rotation_matrix)

        # Create scene
        self.__scene = pyrender.Scene()
        self.__scene.bg_color = [0.5, 0.75, 0.9]
        self.__scene.add(mesh_black, pose=pose_track)
        self.__scene.add(mesh_white, pose=pose_track)
        self.__scene.add(mesh_yellow, pose=pose_track)

        # Add light
        light = pyrender.DirectionalLight(color=[1.0, 1.0, 1.0], intensity=2.0)
        self.__scene.add(light)

    def position(self, x, y, z):
        translation_matrix = tf.translation_matrix([x, y, z])
        self.__camera_pose = np.matmul(self.__camera_pose, translation_matrix)

    def rotate(self, angle, axis):
        if axis == 'x':
            rotation_matrix = tf.rotation_matrix(np.radians(angle), [1, 0, 0])
        elif axis == 'y':
            rotation_matrix = tf.rotation_matrix(np.radians(angle), [0, 1, 0])
        elif axis == 'z':
            rotation_matrix = tf.rotation_matrix(np.radians(angle), [0, 0, 1])
        else:
            raise Exception('Invalid axis')
        self.__camera_pose = np.matmul(self.__camera_pose, rotation_matrix)

    def render(self):
        # Update camera
        camera = pyrender.camera.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=640 / 480)
        if self.__camera_node is not None:
            self.__scene.remove_node(self.__camera_node)
        self.__camera_node = self.__scene.add(camera, pose=self.__camera_pose)

        # Render scene
        r = pyrender.OffscreenRenderer(640, 480)
        color, depth = r.render(self.__scene)
        bytea = color.flatten()
        return PILImage.frombuffer('RGB', (640, 480), bytea, 'raw', 'RGB', 0, 1)


class ImagePublisher:
    publisher = rospy.Publisher('/webcam/image_raw', Image, queue_size=10)
    renderer = None

    def __init__(self, renderer=Renderer()):
        self.renderer = renderer

    def publish(self, image=None):
        if image is None:
            image = self.renderer.render()
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height = image.height
        msg.width = image.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * image.width
        msg.data = np.array(image).tobytes()
        self.publisher.publish(msg)
