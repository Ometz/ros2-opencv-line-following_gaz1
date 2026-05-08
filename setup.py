import os
from glob import glob

from setuptools import setup

package_name = 'line_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aaron S',
    maintainer_email='asoibelman42@gmail.com',
    description='Ros2 Humble OpenCv + Gazebo Split-node line following and branch decision package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_perception_node = line_navigation.line_perception_node:main',
            'line_controller_node = line_navigation.line_controller_node:main',
            'branch_decision_node = line_navigation.branch_decision_node:main',
            'cmd_arbiter_node = line_navigation.cmd_arbiter_node:main',
        ],
    },
)
