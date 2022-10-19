from setuptools import setup

package_name = 'LED_pkg'

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
            'led_node = LED_pkg.led_node:main',
            'led_blink = LED_pkg.led_blink_function:main',
            'led_packet = LED_pkg.led_random_packet_function:main',
        ],
    },
)
