from setuptools import setup
import os
from glob import glob

package_name = 'second_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='firas',
    maintainer_email='firas@example.com',
    description='A simple robot simulation package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol_driver_node = second_package.patrol_driver_node:main',
        ],
    },
)
