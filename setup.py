import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'moni2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rlh',
    maintainer_email='rlh@teknologisk.dk',
    description='Monitor a collection of ROS2 nodes.',
    license='No license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moni2 = moni2.moni2_node:main'
        ],
    },
)