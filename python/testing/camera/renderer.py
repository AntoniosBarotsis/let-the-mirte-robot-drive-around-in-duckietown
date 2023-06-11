import pyrender.material
import trimesh.visual

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
mesh_black_raw = trimesh.load('../towns/duckievoort/black.obj')
mesh_white_raw = trimesh.load('../towns/duckievoort/white.obj')
mesh_yellow_raw = trimesh.load('../towns/duckievoort/yellow.obj')

# Convert & add materials
mesh_black = pyrender.Mesh.from_trimesh(mesh_black_raw, material=mat_black)
mesh_white = pyrender.Mesh.from_trimesh(mesh_white_raw, material=mat_white)
mesh_yellow = pyrender.Mesh.from_trimesh(mesh_yellow_raw, material=mat_yellow)

# Create scene
scene = pyrender.Scene()
scene.add(mesh_black)
scene.add(mesh_white)
scene.add(mesh_yellow)

# Create viewer
pyrender.Viewer(scene, use_raymond_lighting=True)
