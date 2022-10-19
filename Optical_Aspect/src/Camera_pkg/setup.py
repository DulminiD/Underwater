from setuptools import setup

package_name = 'Camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcpslab',
    maintainer_email='mcpslab@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = Camera_pkg.camera_node:main',        
            'oakd_rgb = Camera_pkg.oakd_rgb_function:main',
            'oakd_imu_function = Camera_pkg.oakd_imu_function:main',
            'oakd_imu_rotation_function = Camera_pkg.oakd_imu_rotation_function:main',
            'oakd_stereo = Camera_pkg.oakd_stereo_function:main',
            'oakd_main = Camera_pkg.oakd_main_function:main',
            'oakd_receiver = Camera_pkg.oakd_receiver_function:main'
        ],
    },
)
