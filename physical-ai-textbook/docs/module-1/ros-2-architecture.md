---
title: ROS 2 Architecture
---

# ROS 2 Architecture

The Robot Operating System 2 (ROS 2) provides the foundational communication framework for modern robotics applications. This module introduces the core architectural concepts of ROS 2 and how they enable distributed robotic systems.

## Overview of ROS 2

ROS 2 is a collection of software frameworks that provide operating system-like functionality for robots. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

## Key Architectural Components

### DDS (Data Distribution Service)
- **Purpose**: Provides the underlying communication layer
- **Function**: Enables discovery, serialization, and transport of messages
- **Benefits**: Language and platform independence, real-time capabilities

### Nodes
- **Definition**: Processes that perform computation
- **Communication**: Communicate with other nodes through messages
- **Implementation**: Can be written in C++, Python, or other supported languages

### Topics
- **Function**: Named buses over which nodes exchange messages
- **Pattern**: Publish/Subscribe communication model
- **Usage**: For streaming data between nodes

### Services
- **Function**: Synchronous request/response communication
- **Pattern**: Client/Server communication model
- **Usage**: For remote procedure calls

## Installation and Setup

ROS 2 supports multiple distributions, with the latest being Rolling Ridley. For production systems, it's recommended to use a Long-Term Support (LTS) distribution like Humble Hawksbill.

```bash
# Install ROS 2 Humble Hawksbill (Ubuntu 22.04)
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2