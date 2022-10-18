from setuptools import setup
import os 
import glob
 

package_name = 'oakd'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/RGBandIMU.launch.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcpslab',
    maintainer_email='mcpslab@gmail.com',
    description='Oakd functions',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'oakd_rgb = oakd.oakd_rgb_function:main',
        'oakd_imu_rotation = oakd.oakd_imu_rotation_function:main',
        'oakd_imu_rotation_vector = oakd.oakd_imu_rotation_vector:main',
        'oakd_stereo = oakd.oakd_stereo_function:main',
        'oakd_main = oakd.oakd_main_function:main'
        ],
    },
)
























