from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'go2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Fichiers nécessaires à ROS2 pour retrouver ton package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Fichier package.xml
        ('share/' + package_name, ['package.xml']),

        # === Fichiers de launch ===
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # === Fichiers RViz (.rviz) ===
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

    ],
    install_requires=[
        'setuptools',
        'ament-index-python',
        'launch',
        'launch_ros'
    ],
    zip_safe=True,
    maintainer='go2zilla',
    maintainer_email='go2zilla@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
