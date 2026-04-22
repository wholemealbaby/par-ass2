from setuptools import find_packages, setup
from glob import glob
import os

# Read package name from environment variable with fallback
package_name = os.environ.get('SNC_PACKAGE_NAME', 'snc')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tomg',
    maintainer_email='tomgosling57@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation_node = snc.navigation_node:main',
            'marker_detection_node = snc.marker_detection_node:main',
            'path_tracing_node = snc.path_tracing_node:main',
            'twist_mux = snc.twist_mux:main',
            'best_effort_repeater = snc.best_effort_repeater:main',
            'trigger_listener = snc.trigger_listener:main',
        ],
    },
)
