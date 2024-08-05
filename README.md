# Mars Rover Project

Welcome to the Mars Rover project repository! This project involves designing, fabricating, and testing a Mars Rover capable of autonomous navigation and sample collection. The rover is equipped with a six-wheel drive system, a robust suspension, custom PCBs, and is controlled using the Robot Operating System (ROS).
![WhatsApp Image 2024-08-05 at 05 59 13_29d9cdc6](https://github.com/user-attachments/assets/fbd9cfed-e00a-465d-b8f2-a58f09bdd95c)

## Table of Contents
- [Introduction](#introduction)
- [Hardware Components](#hardware-components)
- [Mechanical Design](#mechanical-design)
- [Electrical Design](#electrical-design)
- [Software Design](#software-design)
- [Installation](#installation)
- [Usage](#usage)
- [Contributors](#contributors)

## Introduction
The Mars Rover project aims to develop a functional rover that can navigate the harsh Martian terrain, collect samples, and perform environmental analysis. This repository contains all the necessary files and instructions to replicate the project.

## Hardware Components
- **DC Motors**: 
  - Used for driving the wheels and operating the robotic arm.
  - Provide sufficient torque and speed to navigate rough terrain and perform precise movements for sample collection.
- **BTS7960 Motor Drivers**: 
  - Control the DC motors with high current capacity.
  - Ensure proper heat dissipation and reliable motor operation.
- **LiPo Battery**: 
  - Powers the entire rover.
  - Provides a high energy density to ensure long operational time.
- **Sensors**: 
  - Methane, hydrogen, and ozone sensors for environmental analysis.
  - Encoder sensors for wheel position feedback and speed measurement.
  - Ultrasonic and infrared sensors for obstacle detection.

## Mechanical Design
- **Six-Wheel Drive with Four-Wheel Steering**: 
  - Ensures stability and maneuverability on uneven terrain.
  - Each wheel is powered independently, allowing for better traction and control.
- **Suspension System**: 
  - Enables traversal over rough surfaces.
  - Absorbs shocks and maintains rover stability, reducing the risk of tipping over.
- **Fabrication Techniques**: 
  - Components are fabricated using 3D printing and laser cutting.
  - Ensures precision, durability, and lightweight construction.

## Electrical Design
- **Custom PCB Design**: 
  - Integrates all electronic components, including sensors, motors, and the power supply.
  - Designed to handle the electrical load and provide reliable connections.
- **Power Management**: 
  - Manages power distribution from the LiPo battery to all components.
  - Includes voltage regulators to ensure consistent and safe power supply.
- **Motor Drivers**: 
  - BTS7960 drivers control the DC motors.
  - Provide high current capability and protect motors from overheating.

## Software Design
- **Robot Operating System (ROS)**: 
  - Used for developing and deploying control software.
  - Provides a flexible framework for writing robot software.
- **Control Algorithms**: 
  - Implemented PID control for motor speed and direction.
  - Ensures precise and smooth control of the rover’s movements.
- **Navigation and Path Planning**: 
  - Developed robust algorithms for autonomous navigation and obstacle avoidance.
  - Includes SLAM (Simultaneous Localization and Mapping) for mapping the environment.
- **Sample Collection Module**: 
  - Programmed 5-DOF robotic arm for precise sample collection.
  - Controlled through ROS, allowing for complex manipulation tasks.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/mars-rover-project.git
   cd mars-rover-project
   ```
2. Install the necessary dependencies:
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-desktop-full
   ```

## Usage
1. Launch the ROS environment:
   ```bash
   source /opt/ros/noetic/setup.bash
   roslaunch mars_rover_bringup mars_rover.launch
   ```
2. Control the rover using the provided scripts or remote control:
   ```bash
   rosrun mars_rover_control rover_control.py
   ```

## Contributors
- **Mohammad Khalifah** - [Mohammad-Khalifah](https://github.com/Mohammad-Khaliafah)

