from setuptools import find_packages, setup
import os
from glob import glob

package_name = "maslab_hardware"

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
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="asa",
    maintainer_email="asapaparo@gmail.com",
    description="TODO: Package description",
    license="MIT",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": ["maslab_robot = maslab_hardware.maslab_robot:main"],
    },
)
