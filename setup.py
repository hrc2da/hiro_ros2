from setuptools import setup
from glob import glob
import os

package_name = 'hiro_dynamixel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name), glob('launch/*launch.py')),
         (os.path.join('share', package_name), glob('config/*config.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='mvl24@cornell.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hiro_arm = hiro_dynamixel.arm_controller:main',
            'hiro_base = hiro_dynamixel.base_controller:main',
            'hiro_control = hiro_dynamixel.robot_controller:main',
            'talker = hiro_dynamixel.speech:main',
            'mover = hiro_dynamixel.target_listener:main',
            'topview = hiro_dynamixel.top_camera:main'
        ],
    },
)
