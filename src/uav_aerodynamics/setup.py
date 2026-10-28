from setuptools import setup
import os
from glob import glob

package_name = 'uav_aerodynamics'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarat_9_7',
    maintainer_email='sarat@uav.org',
    description='Real-time aerodynamic drag and dynamic wind field modeling for UAV flight simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wind_field_node = uav_aerodynamics.wind_field_node:main',
            'aerodynamic_drag_node = uav_aerodynamics.aerodynamic_drag_node:main',
        ],
    },
)
