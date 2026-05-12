# ROS 2 OpenCV Line Following

This repository contains a ROS 2 Humble Python package for camera-based line following in a Gazebo TortoiseBot simulation.

The project was developed while working through The Construct Open Class 190 line-following exercise. The simulation environment and exercise context come from The Construct. This repository focuses on my ROS 2/OpenCV implementation and an extension of the basic approach into a split-node control architecture.

## Overview

The package uses OpenCV to detect a blue line from the robot camera feed and publishes velocity commands through a ROS 2 control pipeline.

The current architecture separates perception, control, branch decision logic, and final command arbitration:

```mermaid
graph TD
    CAM["/camera/image_raw"] --> LP["line_perception_node"]
    CAM --> BD["branch_decision_node"]

    ODOM["/odom"] --> BD

    LP --> LE["/line_error"]
    LP --> LF["/line_found"]

    LE --> LC["line_controller_node"]
    LF --> LC

    LE --> BD
    LF --> BD

    LC --> CVL["/cmd_vel_line"]
    BD --> BB["/branch_bias"]

    CVL --> ARB["cmd_arbiter_node"]
    BB --> ARB

    ARB --> CMD["/cmd_vel"]
