from setuptools import setup
import os
from glob import glob

package_name = 'wheelchair_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/' + package_name, glob(f'{package_name}/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='userpdy606',
    maintainer_email='userpdy606@todo.todo',
    description='TODO: Package description',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = wheelchair_vision.yolo_node:main'
        ],
    },
)