#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

"""Setup file for a ROS 2 package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'doodle_droid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ykechriotis',
    maintainer_email='ykechriotis@student.ethz.ch',
    description='TODO: Package description',
    license='WTFPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processing_node = doodle_droid.process:main',
            'calibrator = doodle_droid.calibrator:main',
            'test_node = doodle_droid.test_node:main',
            'route_planner = doodle_droid.route_planner:main'
        ],
    },
)
