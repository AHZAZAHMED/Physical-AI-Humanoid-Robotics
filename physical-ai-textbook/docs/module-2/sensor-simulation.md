---
title: Sensor Simulation
---

# Sensor Simulation

Sensor simulation is critical for robotics development, allowing for testing and training without physical hardware. This module covers simulating various sensors used in humanoid robotics, including LiDAR, cameras, IMUs, and force/torque sensors.

## Overview of Sensor Simulation

Sensor simulation in robotics environments like Gazebo and Unity allows:
- **Algorithm Development**: Test perception and control algorithms
- **Data Generation**: Create labeled datasets for training
- **Safety Testing**: Evaluate robot behavior in dangerous scenarios
- **Cost Reduction**: Reduce reliance on expensive hardware
- **Repeatability**: Exact reproduction of test scenarios

## Camera Simulation

### Pinhole Camera Model
The pinhole camera model is the foundation for most camera simulations:

```
u = fx * (X/Z) + cx
v = fy * (Y/Z) + cy
```

Where:
- (u, v) are pixel coordinates
- (X, Y, Z) are 3D world coordinates
- fx, fy are focal lengths
- cx, cy are principal point offsets

### Camera Parameters in URDF/SDF

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Camera Calibration
Simulated cameras should match real camera parameters:
- **Intrinsic Parameters**: Focal length, principal point, distortion
- **Extrinsic Parameters**: Position and orientation relative to robot
- **Distortion Models**: Radial and tangential distortion coefficients

## LiDAR Simulation

### Ray-based Simulation
LiDAR sensors are simulated using ray tracing:
- Rays are cast from the sensor origin
- Distance to nearest obstacle is recorded
- Multiple rays form a scan pattern

### LiDAR Configuration in Gazebo

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### Types of LiDAR Simulation
- **2D LiDAR**: Single horizontal plane
- **3D LiDAR**: Multiple planes (Velodyne-style)
- **Solid Angle**: Conical or custom scan patterns

## IMU Simulation

### IMU Components
An IMU typically provides:
- **Accelerometer**: Linear acceleration in 3 axes
- **Gyroscope**: Angular velocity in 3 axes
- **Magnetometer**: Magnetic field (compass heading)

### IMU Simulation in Gazebo

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Force/Torque Sensor Simulation

Force/torque sensors measure forces and torques applied to a robot joint or link:

```xml
<gazebo>
  <joint name="force_torque_joint" type="fixed">
    <parent>parent_link</parent>
    <child>sensor_link</child>
  </joint>

  <sensor name="force_torque_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>sensor_local</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</gazebo>
```

## Sensor Fusion in Simulation

### Combining Multiple Sensors
- **Kalman Filters**: Combine sensor readings optimally
- **Particle Filters**: Handle non-linear sensor models
- **Complementary Filters**: Combine sensors with different characteristics

### Multi-Sensor Integration
- **Temporal Synchronization**: Align sensor timestamps
- **Spatial Calibration**: Transform between sensor frames
- **Covariance Estimation**: Model sensor uncertainties

## Realism Considerations

### Noise Modeling
Real sensors have various types of noise:
- **Gaussian Noise**: Random measurement errors
- **Bias**: Systematic measurement offsets
- **Drift**: Slow changes in bias over time
- **Quantization**: Discrete measurement steps

### Environmental Effects
- **Weather**: Rain, fog, snow affecting sensors
- **Lighting**: Changes in illumination
- **Occlusions**: Objects blocking sensor view
- **Reflections**: Unexpected sensor returns

## Validation and Verification

### Ground Truth Comparison
- **Known Environments**: Compare sensor output to actual values
- **Synthetic Data**: Use known ground truth for perception training
- **Domain Adaptation**: Bridge simulation-to-reality gap

### Sensor-Specific Validation
- **Camera**: Check calibration parameters and distortion
- **LiDAR**: Validate range accuracy and resolution
- **IMU**: Verify noise characteristics and bias
- **Encoders**: Confirm resolution and accuracy

## Best Practices

1. **Parameter Matching**: Match simulated sensor parameters to real hardware
2. **Noise Modeling**: Include realistic noise models
3. **Validation**: Regularly validate simulation against real sensors
4. **Performance**: Balance sensor fidelity with simulation speed
5. **Documentation**: Maintain clear specifications for each sensor model

## Learning Objectives

After completing this module, you will be able to:
- Configure camera, LiDAR, IMU, and other sensor simulations
- Apply appropriate noise models to simulated sensors
- Validate simulated sensors against real hardware
- Implement sensor fusion techniques in simulation
- Consider environmental effects on sensor performance