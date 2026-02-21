# Capstone Design: Vision-Based Semantic Object Tracking Robot
This repository documents the entire development process of my Capstone Design project. The project focuses on integrating Vision (YOLO) and SLAM (ORB-SLAM) to control a TurtleBot for autonomous object tracking and spatial mapping.

## Project Goal
The objective is to build an autonomous system that explores an indoor environment to locate specific target objects (e.g., an apple, desk, or bottle). Once the TurtleBot recognizes the target, it estimates the object's global coordinates, transmits this location data to the user, and autonomously navigates toward the object until it reaches a safe interacting distance.

## System Architecture
The pipeline consists of three core modules:

### 1. Perception (Object Detection)
Tool: YOLOv8

Description: The robot processes the real-time camera feed to detect target objects. YOLO generates bounding boxes around the identified objects, acting as the primary trigger for the tracking sequence.

### 2. Control (Visual Servoing & Approach)
Description: Once the target is detected, the system applies a visual servoing control logic. By calculating the error between the bounding box center and the camera frame's center, it dynamically adjusts the TurtleBot's angular and linear velocity (cmd_vel).

The robot moves forward while keeping the object centered, stopping automatically at a predefined distance (e.g., 0.3m - 0.5m).

### 3. Mapping & Localization (ORB-SLAM)
Tool: ORB-SLAM

Description: To provide spatial context, the system runs ORB-SLAM to build a map of the environment and continuously track the robot's current pose.

By fusing the robot's pose from SLAM with the visual data from YOLO, the system estimates the target object's 3D coordinates and alerts the user with the precise location.
