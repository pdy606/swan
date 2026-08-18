import os
from glob import glob
from setuptools import setup

package_name = 'swan_pipeline'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models', 'illegal_car'), glob('models/illegal_car/*')),
        (os.path.join('share', package_name, 'models', 'moving_pedestrian'), glob('models/moving_pedestrian/*')),
        (os.path.join('share', package_name, 'models', 'wheelchair'), glob('models/wheelchair/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swan',
    maintainer_email='swan@swan.local',
    description='SWAN 인지-판단-제어 파이프라인 스켈레톤',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_node = swan_pipeline.perception_stub:main',
            'gt_perception_node = swan_pipeline.gt_perception:main',
            'lidar_perception_node = swan_pipeline.lidar_perception_node:main',
            'scan_noise_node = swan_pipeline.scan_noise_node:main',
            'control_node = swan_pipeline.control_node:main',
            'assist_node = swan_pipeline.assist_node:main',
            'virtual_user = swan_pipeline.virtual_user:main',
            'wheelchair_teleop = swan_pipeline.wheelchair_teleop:main',
            'hw_bridge_node = swan_pipeline.hw_bridge_node:main',
            'camera_perception_node = swan_pipeline.camera_perception_node:main',
            'fusion_node = swan_pipeline.fusion_node:main',
            'yolo_adapter_node = swan_pipeline.yolo_adapter_node:main',
            'scenario_logger = swan_pipeline.scenario_logger:main',
            'jerk_logger = swan_pipeline.jerk_logger:main',
            'brake_test = swan_pipeline.brake_test:main',
        ],
    },
)
