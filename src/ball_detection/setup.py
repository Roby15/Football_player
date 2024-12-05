from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'ball_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install the package.xml file
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch',
            glob('launch/ball.py')),
    ],
    install_requires=['setuptools','opencv-python'],
    zip_safe=True,
    maintainer='roby',
    maintainer_email='roby@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['ball_detector = ball_detection.ball_detector:main',
            'ball_follower = ball_detection.ball_follower:main',
            'ball_finder=ball_detection.ball_finder:main'],
    },
)
