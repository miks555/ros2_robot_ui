# UR5 Robot Driver and Black Box UI Guide

This guide explains how to launch the UR5 robot driver, access the VNC interface, and run the Black Box UI for controlling the robot.

## 1. Start Docker Daemon

Make sure Docker is running:

sudo dockerd

## 2. Launch URSim for UR5

Start the URSim simulation environment:

ros2 run ur_client_library start_ursim.sh -m ur5

## 3. Launch the UR Robot Driver

Run the UR Robot Driver with RViz enabled:

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.56.101 launch_rviz:=true

## 4. Access VNC Interface

Open Google Chrome (or any browser) and navigate to the VNC interface:

google-chrome http://192.168.56.101:6080/vnc.html

Perform the robot homing procedure via the VNC interface.

## 5. External Control via Console

Once homing is completed, you can execute external control commands from the terminal to control the robot.

## 6. Launch the Black Box UI

Run the Black Box user interface to interact with the robot:

ros2 launch black_box robot_ui_launch.py

## Notes

- The Black Box UI allows clicking on a virtual window to send commands to the robot.
- Make sure all ROS2 nodes and topics are properly initialized before using the UI.
- Ensure that `/scaled_joint_trajectory_controller/joint_trajectory` is active for the robot to respond.

