import os
from glob import glob

from setuptools import find_packages, setup

package_name = "vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="asa",
    maintainer_email="asapaparo@gmail.com",
    description="TODO: Package description",
    license="MIT",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": ["cube_detect = vision.cube_detect:main"],
    },
)
