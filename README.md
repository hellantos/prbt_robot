# PRBT Robot ROS2 support
This repository contains basic ROS2 support for the Pilz PRBT robot.

## Usage
Currently this has only been tested with canopen_fake_slaves/cia402_slave as hardware emulation.
To run this test do the following:

1. Setup vcan0
2. Launch vcan.launch.py of prbt_robot_moveit_config
    ```
    ros2 launch prbt_robot_moveit_config vcan.launch.py
    ```

3. Enable all joints
    ```
    ros2 service call prbt_joint_1_controller/init std_srvs/srv Trigger
    ros2 service call prbt_joint_2_controller/init std_srvs/srv Trigger
    ros2 service call prbt_joint_3_controller/init std_srvs/srv Trigger
    ros2 service call prbt_joint_4_controller/init std_srvs/srv Trigger
    ros2 service call prbt_joint_5_controller/init std_srvs/srv Trigger
    ros2 service call prbt_joint_6_controller/init std_srvs/srv Trigger
    ```

4. Switch to position mode for all joints
    ```
    ros2 service call prbt_joint_1_controller/position_mode std_srvs/srv Trigger
    ros2 service call prbt_joint_2_controller/position_mode std_srvs/srv Trigger
    ros2 service call prbt_joint_3_controller/position_mode std_srvs/srv Trigger
    ros2 service call prbt_joint_4_controller/position_mode std_srvs/srv Trigger
    ros2 service call prbt_joint_5_controller/position_mode std_srvs/srv Trigger
    ros2 service call prbt_joint_6_controller/position_mode std_srvs/srv Trigger
    ```

5. Plan and execute trajectories in rviz or with moveit
