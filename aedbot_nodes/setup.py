import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = "aedbot_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=[]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    author="Bishop Pearson",
    author_email="bishopearson@gmail.com",
    maintainer="Bishop Pearson",
    maintainer_email="bishopearson@gmail.com",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description=(
        "aedbot_nodes that include iff drive controller, odometry and tf node"
    ),
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_subscriber = scripts.imu_subscriber:main",
            "get_dest = scripts.get_dest:main",
            "test_sub = scripts.test_sub:main",
            "play_siren = scripts.play_siren:main",
            "HRI = scripts.HRI:main",
            "cpr_node = scripts.cpr_node:main",
        ],
    },
)
