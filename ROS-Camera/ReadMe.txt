OAKD CAMERA ROS Implementation 

Requirements 

1. ROS2 version Galactic
2. Ubuntu version >=18.0.0
3. Python3
4. Depthai 
    1. Check the link for installation guidelines - https://github.com/luxonis/depthai

Implementation

1. Open a terminal in the respective main folder 
2. Install the setup bash by " . install/setup.bash"
3. Install the local setup bash if you have not installed it before " . install/local_setup.bash"
4. Build the oakd package using "colcon build --packages-select oakd"

Nodes 

1. Run the main node by "ros2 run oakd <<node_name>>"
2. Main Node - (RGB Video and IMU details both) "ros2 run oakd oakd_main"
3. RGB camera "ros2 run oakd oakd_rgb"
4. IMU details "ros2 run oakd oakd_imu_rotation" and "ros2 run oakd oakd_imu_rotation"
5. Stereo camera "ros2 run oakd oakd_stereo"



