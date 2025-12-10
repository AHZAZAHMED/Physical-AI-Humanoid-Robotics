---
title: Nav2 and Bipedal Movement
---

# Nav2 and Bipedal Movement

Navigation 2 (Nav2) is the navigation stack for ROS 2, providing path planning, obstacle avoidance, and navigation capabilities. This module covers implementing navigation for bipedal humanoid robots, addressing the unique challenges of legged locomotion.

## Introduction to Navigation 2 (Nav2)

Nav2 provides:
- **Path Planning**: Global and local path planning algorithms
- **Obstacle Avoidance**: Dynamic obstacle detection and avoidance
- **Localization**: AMCL and other localization methods
- **Recovery Behaviors**: Strategies for handling navigation failures
- **Behavior Trees**: Task planning and execution

### Nav2 Architecture
- **Global Planner**: Creates optimal path from start to goal
- **Local Planner**: Executes path while avoiding obstacles
- **Controller**: Converts plan to robot commands
- **Costmap**: Represents obstacles and navigation space
- **Recovery**: Handles navigation failures

## Challenges in Bipedal Navigation

### Stability Considerations
- **Center of Mass**: Maintaining balance during movement
- **Foot Placement**: Precise footstep planning
- **Dynamic Balance**: Real-time balance control
- **Terrain Adaptation**: Adapting to uneven surfaces

### Navigation Constraints
- **Step Size**: Limited by leg length and joint ranges
- **Turning Radius**: Constrained by bipedal kinematics
- **Speed Limitations**: Balance constraints on velocity
- **Terrain Requirements**: Need for stable foot placement

## Nav2 for Bipedal Robots

### Custom Costmaps
Bipedal robots require specialized costmaps:

```yaml
# Bipedal costmap configuration
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: true
  rolling_window: false
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 5.0
  static_map: false
  rolling_window: true
  width: 6.0
  height: 6.0
  resolution: 0.05
  plugins:
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: voxel_layer, type: "nav2_costmap_2d::VoxelLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
```

### Bipedal-Specific Parameters
```yaml
# Controller configuration for bipedal robots
controller_server:
  ros__parameters:
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.01
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.1
      wz_std: 0.3
      vx_max: 0.5
      vx_min: -0.2
      vy_max: 0.3
      wz_max: 0.5
      sim_period: 0.05
      speed_regulation_factor: 0.0
      reference_heading_factor: 1.0
      track_arc_factor: 2.0
      goal_dist_deadband: 0.1
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.02
      short_circuit_moving_average_length: 5
      moving_average_length: 15
      max_iterations: 3
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDriveMotionModel"
```

## Footstep Planning

### Overview
Footstep planning is critical for bipedal navigation, determining where and when to place feet.

### Key Considerations
- **Stability**: Maintaining center of mass over support polygon
- **Terrain**: Accounting for surface properties and obstacles
- **Kinematics**: Respecting joint limits and step constraints
- **Dynamics**: Managing momentum and balance

### Footstep Planning Algorithms
1. **A* Search**: Grid-based search for feasible footsteps
2. **RRT**: Rapidly-exploring random trees for complex terrain
3. **Model Predictive Control**: Optimization-based planning
4. **Learning-based**: Data-driven approaches

## Balance Control Integration

### Zero Moment Point (ZMP)
The ZMP is crucial for bipedal stability:

```cpp
// ZMP calculation example
class BalanceController
{
public:
    geometry_msgs::msg::Point calculateZMP(
        const std::vector<double>& com_pos,
        const std::vector<double>& com_vel,
        const std::vector<double>& com_acc)
    {
        geometry_msgs::msg::Point zmp;
        double gravity = 9.81;
        double com_height = com_pos[2];

        zmp.x = com_pos[0] - (com_acc[0] * com_height) / gravity;
        zmp.y = com_pos[1] - (com_acc[1] * com_height) / gravity;

        return zmp;
    }
};
```

### Capture Point
The capture point indicates where to step to stop:

```cpp
// Capture point calculation
std::pair<double, double> calculateCapturePoint(
    const std::vector<double>& com_pos,
    const std::vector<double>& com_vel)
{
    double omega = sqrt(9.81 / com_pos[2]); // Natural frequency
    double capture_x = com_pos[0] + com_vel[0] / omega;
    double capture_y = com_pos[1] + com_vel[1] / omega;

    return std::make_pair(capture_x, capture_y);
}
```

## Bipedal-Specific Navigation Controllers

### Walking Pattern Generators
- **Preview Control**: Uses future reference trajectory
- **Linear Inverted Pendulum**: Simplified balance model
- **Cart-Table**: Extended LIP model with variable height

### Gait Adaptation
Controllers must adapt to different walking patterns:
- **Static Walking**: Stable at each step
- **Dynamic Walking**: Continuous momentum
- **Transition Steps**: Turning and speed changes

## Nav2 Behavior Trees for Bipedal Robots

### Custom Behavior Tree Nodes
Bipedal robots may need specialized behavior tree nodes:

```xml
<BehaviorTree>
  <Sequence>
    <GoalUpdated/>
    <ComputePathToPose goal="{goal}" path="{path}"/>
    <Fallback>
      <RecoveryNode number_of_retries="2">
        <Sequence>
          <SmoothPath path="{path}" output="{smoothed_path}"/>
          <FollowPath path="{smoothed_path}"/>
        </Sequence>
        <ReactiveFallback>
          <CheckFootPlacement path="{smoothed_path}"/>
          <FootstepPlanning path="{smoothed_path}"/>
        </ReactiveFallback>
      </RecoveryNode>
      <GoalReached goal="{goal}"/>
    </Fallback>
  </Sequence>
</BehaviorTree>
```

### Bipedal Recovery Behaviors
- **Footstep Adjustment**: Modify path for better foot placement
- **Balance Recovery**: Regain balance before continuing
- **Safe Stopping**: Stable stop in hazardous situations

## Integration with Humanoid Control Systems

### Whole-Body Control
Navigation commands must integrate with whole-body controllers:

```cpp
class BipedalNavigator
{
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    std::shared_ptr<WholeBodyController> wb_controller_;

public:
    void executeNavigation(nav_msgs::msg::Path::SharedPtr path)
    {
        // Plan footsteps along path
        std::vector<Footstep> footsteps = planFootsteps(*path);

        // Generate whole-body motion
        for (const auto& step : footsteps) {
            // Plan step motion
            trajectory_msgs::msg::JointTrajectory step_traj =
                wb_controller_->planStepMotion(step);

            // Execute step
            joint_traj_pub_->publish(step_traj);

            // Wait for completion
            waitForExecution();
        }
    }
};
```

### Balance Integration
Navigation commands must consider balance constraints:

```python
def navigate_with_balance(robot_pose, goal_pose, balance_controller):
    # Plan path considering balance constraints
    path = plan_balanced_path(robot_pose, goal_pose)

    # Execute with balance feedback
    for waypoint in path:
        # Check balance before moving
        if not balance_controller.is_stable():
            balance_controller.regain_balance()

        # Move to waypoint
        robot.move_to(waypoint)

        # Monitor balance during movement
        balance_controller.update()
```

## Simulation and Testing

### Gazebo Integration
Testing navigation in simulation before deployment:

```xml
<!-- Bipedal robot navigation plugin -->
<gazebo>
  <plugin name="bipedal_nav_plugin" filename="libbipedal_nav_plugin.so">
    <robot_base_frame>base_link</robot_base_frame>
    <odom_frame>odom</odom_frame>
    <foot_frames>left_foot_link,right_foot_link</foot_frames>
    <balance_controller>
      <kp>10.0</kp>
      <ki>0.1</ki>
      <kd>0.5</kd>
    </balance_controller>
  </plugin>
</gazebo>
```

### Isaac Sim for Navigation
Using Isaac Sim for complex navigation scenarios:

```python
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.sensor import ContactSensor

# Set up navigation testing environment
def setup_navigation_test_env():
    # Create varied terrain
    create_rough_terrain()
    create_obstacles()

    # Add sensors for navigation
    add_contact_sensors()
    add_vision_sensors()

    # Configure physics for bipedal walking
    set_friction_properties()
```

## Performance Evaluation

### Metrics for Bipedal Navigation
- **Navigation Success Rate**: Successful goal achievement
- **Path Efficiency**: Path length vs. optimal path
- **Balance Maintenance**: Balance error during navigation
- **Computation Time**: Real-time performance
- **Energy Efficiency**: Power consumption during navigation

### Benchmarking
Testing on standardized scenarios:
- **Flat Terrain**: Basic navigation capability
- **Uneven Terrain**: Adaptation to surface variations
- **Obstacle Avoidance**: Dynamic obstacle handling
- **Stair Navigation**: Complex terrain traversal

## Best Practices

1. **Stability First**: Prioritize balance over navigation speed
2. **Conservative Planning**: Account for balance constraints
3. **Fallback Strategies**: Have safe stopping procedures
4. **Simulation Testing**: Extensive testing before real robot deployment
5. **Gradual Complexity**: Increase difficulty progressively

## Learning Objectives

After completing this module, you will be able to:
- Configure Nav2 for bipedal robot navigation
- Implement footstep planning algorithms
- Integrate balance control with navigation
- Create custom behavior trees for bipedal navigation
- Evaluate and optimize bipedal navigation performance