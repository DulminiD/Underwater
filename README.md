# Underwater

OAKD CAMERA ROS Implementation 

## Requirements 

1. ROS2 version Galactic
2. Ubuntu version >=18.0.0
3. Python3
4. Depthai 
    i. Check the link for installation guidelines - https://github.com/luxonis/depthai

## Implementation

There are two packages in the optical-aspect workspace. 

1. LED package 

This package is used to maintain the LED or the transmitter of the system

I. Open a terminal in the "Optical-Aspect" folder.
II. Build the LED package "colcon build --packages-select LED_pkg". 
III. Run commands " . install/setup.bash" and ". install/local_setup.bash" to add the workspace to the path 

There are 2 scripts. 

'''

    i. led_blink_function - Sending ON and OFF sigals to the LED 
        
        Run using command
        ros2 run LED_pkg led_blink

    ii. led_random_packet_function - Sending a random packet of signal to the LED 

        Run using command
        ros2 run LED_pkg led_packet

'''

2. Camera package 

This package is used to maintain the camera or the receiver of the system 

I. Open a terminal in the "Optical-Aspect" folder.
II. Build the LED package "colcon build --packages-select Camera_pkg". 
III. Run commands " . install/setup.bash" and ". install/local_setup.bash" to add the workspace to the path 

There are 6 scripts. 

'''

    i. oakd_rgb_function - Turns on the RGB camera  
        
        Run using command
        ros2 run Camera_pkg oakd_rgb

    ii. oakd_imu_function - Collect information from IMU details from accelerator and gyrometer of the oakd camera 

        Run using command
        ros2 run Camera_pkg oakd_imu_rotation

    iii. oakd_imu_rotation_function - Collect information from IMU rotation functions of the oakd camera 

        Run using command
        ros2 run Camera_pkg oakd_imu_rotation_function

    iv. oakd_stereo_function - Turns on the stereo cameras
        
        Run using command
        ros2 run Camera_pkg oakd_stereo

    v. oakd_main_function - Turns on the RGB camera saving a video and collects IMU details 
        
        Run using command
        ros2 run Camera_pkg oakd_main

    vi. oakd_receiver_function - Idenities whether the light in ON or OFF
        
        Run using command
        ros2 run Camera_pkg oakd_receiver

'''




