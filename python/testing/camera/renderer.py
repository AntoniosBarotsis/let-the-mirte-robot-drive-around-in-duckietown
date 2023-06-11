import pyrender.material
import trimesh.visual
import numpy as np
import transformations as tf
import matplotlib.pyplot as plt


class Renderer:
    camera_pose = None
    scene = None

    def __init__(self, scenario='duckievoort'):
        # Initialize camera pose
        self.camera_pose = tf.identity_matrix()
        rotation_matrix = tf.rotation_matrix(np.radians(90), [1, 0, 0])  # Parallel to the ground
        self.camera_pose = np.matmul(self.camera_pose, rotation_matrix)

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
            mesh_black_raw = trimesh.load('../towns/duckievoort/black.obj')
            mesh_white_raw = trimesh.load('../towns/duckievoort/white.obj')
            mesh_yellow_raw = trimesh.load('../towns/duckievoort/yellow.obj')
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
        self.scene = pyrender.Scene()
        self.scene.bg_color = [0.5, 0.75, 0.9]
        self.scene.add(mesh_black, pose=pose_track)
        self.scene.add(mesh_white, pose=pose_track)
        self.scene.add(mesh_yellow, pose=pose_track)

    def position(self, x, y, z):
        translation_matrix = tf.translation_matrix([x, y, z])
        self.camera_pose = np.matmul(self.camera_pose, translation_matrix)

    def rotate(self, angle, axis):
        if axis == 'x':
            rotation_matrix = tf.rotation_matrix(np.radians(angle), [1, 0, 0])
        elif axis == 'y':
            rotation_matrix = tf.rotation_matrix(np.radians(angle), [0, 1, 0])
        elif axis == 'z':
            rotation_matrix = tf.rotation_matrix(np.radians(angle), [0, 0, 1])
        else:
            raise Exception('Invalid axis')
        self.camera_pose = np.matmul(self.camera_pose, rotation_matrix)

    def render(self):
        # Add camera
        camera = pyrender.camera.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=640 / 480)
        self.scene.add(camera, pose=self.camera_pose)

        # Render scene
        pyrender.viewer.Viewer(self.scene, use_raymond_lighting=True)


renderer = Renderer()
renderer.render()
