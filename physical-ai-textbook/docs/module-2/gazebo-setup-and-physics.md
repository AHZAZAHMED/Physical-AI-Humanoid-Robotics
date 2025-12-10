---
title: Gazebo Setup and Physics
---

# Gazebo Setup and Physics

Gazebo is a powerful 3D simulation environment that provides realistic physics simulation for robotics development. This module covers setting up Gazebo and understanding its physics engine for humanoid robotics applications.

## Introduction to Gazebo

Gazebo provides:
- **Realistic Physics**: Accurate simulation of rigid body dynamics
- **High-Fidelity Rendering**: Detailed visual simulation
- **Sensor Simulation**: Cameras, LiDAR, IMUs, and other sensors
- **Plugin Architecture**: Extensible functionality through plugins
- **ROS Integration**: Seamless integration with ROS 2

## Installation

### Installing Gazebo Garden (Latest Version)

```bash
# Install Gazebo Garden
sudo apt update
sudo apt install gz-garden

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-*
```

## Basic Gazebo Concepts

### Worlds
Worlds define the environment in which robots operate:
- Lighting conditions
- Physics parameters
- Static objects
- Terrain

### Models
Models represent objects in the simulation:
- Robots
- Static objects
- Dynamic objects

### Plugins
Plugins extend Gazebo functionality:
- Sensor plugins
- Controller plugins
- Physics plugins

## Physics Engine Configuration

Gazebo supports multiple physics engines, with Ignition Physics providing a unified interface.

### Physics Parameters

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

### Key Parameters:
- **max_step_size**: Simulation time step (smaller = more accurate but slower)
- **real_time_factor**: Target simulation speed (1.0 = real-time)
- **gravity**: Gravitational acceleration vector

## Setting up a Robot in Gazebo

### Launch File Example

```xml
<launch>
  <!-- Start Gazebo server -->
  <node name='gazebo' pkg='gazebo_ros' exec='gzserver' args='$(find-pkg-share my_robot_gazebo)/worlds/empty.sdf'/>

  <!-- Start Gazebo client -->
  <node name='gzclient' pkg='gazebo_ros' exec='gzclient' required='true'/>

  <!-- Spawn robot in Gazebo -->
  <node name='spawn_entity' pkg='gazebo_ros' exec='spawn_entity.py'
        args='-topic robot_description -entity my_robot'/>
</launch>
```

## Sensor Simulation

### Camera Sensor
```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### LiDAR Sensor
```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1.0</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
</sensor>
```

## Physics Considerations for Humanoid Robots

### Stability Challenges
- **Center of Mass**: Critical for balance in bipedal robots
- **Foot Contact**: Modeling ground contact for walking
- **Inertial Properties**: Accurate mass and inertia for control

### Tuning Parameters
- **Damping**: Helps with stability
- **Friction**: Affects walking dynamics
- **Contact Stiffness**: Affects collision response

## Best Practices

1. **Model Simplification**: Use simplified collision models for better performance
2. **Parameter Tuning**: Match simulation parameters to real robot when possible
3. **Validation**: Compare simulation and real robot behavior
4. **Performance**: Balance accuracy with simulation speed
5. **Debugging**: Use Gazebo's visualization tools for debugging

## Learning Objectives

After completing this module, you will be able to:
- Install and configure Gazebo for ROS 2
- Create and configure simulation worlds
- Integrate robots with Gazebo simulation
- Configure physics parameters for humanoid robots
- Implement sensor simulation for robotics applications