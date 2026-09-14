import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swan_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gyuwon',
    maintainer_email='gwseo0909@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'situation_node = swan_pipeline.situation_node:main',
            'fusion_node = swan_pipeline.fusion_node:main',
            'situation_c_node = swan_pipeline.situation_c_node:main',
            'signal_node = swan_pipeline.signal_node:main',
        ],
    },
)
