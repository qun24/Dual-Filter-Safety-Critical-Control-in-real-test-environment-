# Obstacle Avoidance Robot Project in Real Environment

This program utilizes a self-built robot to perform obstacle avoidance in different real-world scenarios. (For reference only for students who will continue to use the original robot). The entire obstacle avoidance program is developed under ROS2.

## Robot Startup Steps

1. SSH into the robot
```
ssh orangepi@192.168.1.101
cd ros2_ws/
source install/setup.bash
export ROS_DOMAIN_ID=30
```

2. Launch relevant nodes
- Launch the camera
```
ros2 launch realsense2_camera rs_launch.py
```

- Compress images  
```
ros2 run image_compress image_compress_node
```

- Launch odom
```
ros2 launch bno055 data_collect_launch.py
```

- Launch robot controller
```
bash script/sudo_run_controller.sh
```
Use the gamepad, press y to switch to accepting node commands, press x to switch to manual control.

## Running the Obstacle Avoidance Program

### 1. Static and Dynamic Environment Avoidance

- Enter the package and launch salient object detection
```
ros2 launch sod_launch preparing_sod.launch.py
```

- Run obstacle point cloud filtering, information extraction, etc.
```
ros2 launch sod_launch obs_pcl_handel.launch.py
```

- Run the obstacle avoidance program
```
export ROS_DOMAIN_ID=30
ros2 run move_obs_avoidence move_obs_avoidence_node
```

Tip: To view in Rviz2, set `export ROS_DOMAIN_ID=30` first.

### 2. Path Following Environment Avoidance 

- Enter the package and launch salient object detection
```
ros2 launch sod_launch preparing_sod.launch.py
```

- Run obstacle point cloud filtering, information extraction, trajectory generation, etc.
```
ros2 launch sod_launch path_following_preparing.launch.py
```

- Run the obstacle avoidance program
```
ros2 run path_following_with_obstacles elliptical_path_following_typeII-d-zcbfs_avoidence.py
```
