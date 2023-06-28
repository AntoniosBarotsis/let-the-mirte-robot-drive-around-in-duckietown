from setuptools import setup


setup(
    name="mirte",
    version="1.0",
    install_requires=[
        "opencv-python",
        "numpy",
        "rospy",
        "pyyaml",
        "rospkg",
        "cvbridge3",
    ],
    packages=["mirte_duckietown"],
    package_data={"mirte_duckietown": ["*.yaml", "*.yml"]},
    include_package_data=True,
)