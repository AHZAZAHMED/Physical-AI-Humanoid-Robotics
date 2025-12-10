---
title: URDF and Packages
---

# URDF and Packages

Unified Robot Description Format (URDF) and ROS packages form the foundation for describing robots and organizing code in ROS 2. This module covers how to create robot descriptions and organize your code into reusable packages.

## URDF (Unified Robot Description Format)

URDF is an XML format for representing a robot model. It describes the robot's physical and visual properties, including links, joints, and inertial characteristics.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="example_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.0 0.2 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### URDF Elements

- **Links**: Rigid bodies with visual, collision, and inertial properties
- **Joints**: Connections between links with specific degrees of freedom
- **Materials**: Visual appearance properties
- **Gazebo Plugins**: Simulation-specific extensions

## ROS Packages

Packages are the basic building blocks of ROS software organization.

### Package Structure

```
my_robot_package/
├── CMakeLists.txt
├── package.xml
├── launch/
├── src/
├── include/
├── config/
├── meshes/
├── urdf/
└── worlds/
```

### Creating a Package

```bash
# Create a new package
ros2 pkg create --build-type ament_python my_robot_package
```

### package.xml

The package manifest file:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>Example robot package</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Xacro for Complex URDF

Xacro is an XML macro language that extends URDF with features like variables, math expressions, and macros.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_robot">
  <xacro:property name="base_width" value="0.5"/>
  <xacro:property name="base_length" value="0.5"/>
  <xacro:property name="base_height" value="0.2"/>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Best Practices

1. **Modularity**: Break complex robots into logical subsystems
2. **Parameterization**: Use xacro for complex robot descriptions
3. **File Organization**: Keep URDF files in the urdf/ directory
4. **Documentation**: Include clear comments in URDF files
5. **Validation**: Test URDF files with robot_state_publisher

## Learning Objectives

After completing this module, you will be able to:
- Create and validate URDF files for robot models
- Organize code into ROS 2 packages
- Use Xacro to simplify complex robot descriptions
- Apply best practices for robot modeling