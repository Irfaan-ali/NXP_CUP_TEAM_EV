import os
from glob import glob
from setuptools import setup, find_packages

package_name = "b3rb_ros_line_follower"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        package_name: [
            "*.pt",
            "*.h5",
        ],
    },

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    team_name="ElectroVerse",
    team_id="3575",

    description="Autonomous mission controller and navigation package for the NXP Cup India 2026 B3RB Buggy from Team ElectroVerse - 3575",

    tests_require=["pytest"],

    entry_points={
        "console_scripts": [

            # Lane Detection
            "vectors = b3rb_ros_line_follower.b3rb_ros_edge_vectors:main",

            # Main Runner
            "runner = b3rb_ros_line_follower.b3rb_ros_line_follower:main",

            # Sign Detection
            "detect = b3rb_ros_line_follower.b3rb_ros_object_recog:main",

            # QR Detection
            "qr_detect = b3rb_ros_line_follower.b3rb_ros_qr_detector:main",
        ],
    },
)