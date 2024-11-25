from setuptools import find_packages, setup

package_name = 'doodle_droid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 
                                   'launch/camera.launch.xml',
                                   'launch/calibrator.launch.xml',
                                   'config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ykechriotis',
    maintainer_email='ykechriotis@student.ethz.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processing_node = doodle_droid.process:main',
            'calibrator = doodle_droid.calibrator:main'
        ],
    },
)
