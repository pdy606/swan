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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swan',
    maintainer_email='swan@swan.local',
    description='SWAN 통합 파이프라인 (퓨전 + 상황판단 B/C)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fusion_node = swan_pipeline.fusion_node:main',       # 퍼셉션 코어(Camera+LiDAR)
            'situation_node = swan_pipeline.situation_node:main',  # C: 물리 장애물 회피
            'signal_node = swan_pipeline.signal_node:main',        # B: 신호 의미기반
        ],
    },
)
