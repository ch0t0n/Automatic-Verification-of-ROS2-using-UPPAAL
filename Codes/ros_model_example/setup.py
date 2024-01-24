import os
from glob import glob
from setuptools import setup

package_name = 'ros_model_example'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch_file.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='choton@ksu.edu',
    description='ROS model example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher1 = ros_model_example.publisher1:main',
            'subscriber1 = ros_model_example.subscriber1:main',
            'subscriber2 = ros_model_example.subscriber2:main',
        ],
    },
)
