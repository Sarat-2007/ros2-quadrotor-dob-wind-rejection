from setuptools import setup
import os
from glob import glob

package_name = 'uav_dashboard'

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
    description='Real-time interactive control panel, telemetry monitor, and experiment logger for UAV simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'live_tuning_gui = uav_dashboard.live_tuning_gui:main',
            'uav_teleop_gui = uav_dashboard.uav_teleop_gui:main',
        ],
    },

)
