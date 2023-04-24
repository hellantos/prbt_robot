# PRBT Robot ROS2 support
This repository contains basic ROS2 support for the Pilz PRBT robot.

## Working with real hardware

### Initial setup
Check robot is configured correctly:
- `ros2 launch prbt_robot_support prbt_robot.launch.py`
- Initialize all joints
- Switch to position mode for all joints
    ```bash
    ros2 service call /prbt_joint_1/init std_srvs/srv/Trigger
    ros2 service call /prbt_joint_2/init std_srvs/srv/Trigger
    ros2 service call /prbt_joint_3/init std_srvs/srv/Trigger
    ros2 service call /prbt_joint_4/init std_srvs/srv/Trigger
    ros2 service call /prbt_joint_5/init std_srvs/srv/Trigger
    ros2 service call /prbt_joint_6/init std_srvs/srv/Trigger
    ```
- Output from joint initialization successful

    ```bash
    [device_container_node-1] [INFO] [1682344012.883996069] [prbt_joint_6]: Initialised object: node_id X, index 6041, subindex 0, data 37415, RPDO: yes, TPDO: no
    [device_container_node-1] [INFO] [1682344012.885767445] [prbt_joint_6]: Initialised object: node_id X, index 6040, subindex 0, data 271, RPDO: no, TPDO: yes
    [device_container_node-1] [INFO] [1682344012.885810730] [prbt_joint_6]: Initialised object: node_id X, index 6061, subindex 0, data 7, RPDO: yes, TPDO: no
    [device_container_node-1] [INFO] [1682344012.889486704] [prbt_joint_6]: Initialised object: node_id X, index 6060, subindex 0, data 0, RPDO: no, TPDO: yes
    [device_container_node-1] [INFO] [1682344012.891355262] [prbt_joint_6]: Initialised object: node_id X, index 6502, subindex 0, data 67, RPDO: no, TPDO: no
    [device_container_node-1] [INFO] [1682344012.893113122] [prbt_joint_6]: Initialised object: node_id X, index 60c1, subindex 1, data 0, RPDO: no, TPDO: yes
    [device_container_node-1] [INFO] [1682344012.894783925] [prbt_joint_6]: Initialised object: node_id X, index 6042, subindex 0, data 0, RPDO: no, TPDO: no
    [device_container_node-1] [INFO] [1682344012.896484203] [prbt_joint_6]: Initialised object: node_id X, index 607a, subindex 0, data 0, RPDO: no, TPDO: yes
    ```

@note: Following
- Index: 6041, subindex: 0, data: 37415 --> 0x9227 --> 0b1001001000100111 --> Statusword: Operation enabled
- Index: 6061, subindex: 0, data: 7 --> Mode of operation: Interpolated position mode.

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

    ```bash
    ros2 service call /prbt_joint_1/position_mode std_srvs/srv/Trigger
    ros2 service call /prbt_joint_2/position_mode std_srvs/srv/Trigger
    ros2 service call /prbt_joint_3/position_mode std_srvs/srv/Trigger
    ros2 service call /prbt_joint_4/position_mode std_srvs/srv/Trigger
    ros2 service call /prbt_joint_5/position_mode std_srvs/srv/Trigger
    ros2 service call /prbt_joint_6/position_mode std_srvs/srv/Trigger
    ```

5. Plan and execute trajectories in rviz or with moveit
