---
title: Humanoid Kinematics and Control
---

# Humanoid Kinematics and Control

Humanoid kinematics and control form the foundation for enabling human-like movement and interaction in robotic systems. This module covers the mathematical foundations, control strategies, and implementation techniques for humanoid robot motion.

## Introduction to Humanoid Kinematics

Humanoid kinematics involves:
- **Forward Kinematics**: Computing end-effector positions from joint angles
- **Inverse Kinematics**: Computing joint angles from desired end-effector positions
- **Jacobian Computation**: Relating joint velocities to end-effector velocities
- **Dynamics**: Understanding forces and torques in motion
- **Balance Control**: Maintaining stability during movement

## Humanoid Robot Kinematic Structure

### Degrees of Freedom
Humanoid robots typically have:
- **Legs**: 6+ DOF each (hip, knee, ankle)
- **Arms**: 7+ DOF each (shoulder, elbow, wrist)
- **Torso**: 2-3 DOF for upper body movement
- **Head**: 2-3 DOF for gaze control
- **Total**: 30+ DOF for full humanoid

### Common Configurations
- **NAO**: 25 DOF, small humanoid
- **Pepper**: 20 DOF, social robot
- **ATLAS**: 28+ DOF, large humanoid
- **Honda ASIMO**: 57 DOF, advanced humanoid

## Forward Kinematics

### Mathematical Representation
Forward kinematics computes the position and orientation of end-effectors given joint angles:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def dh_transform(a, alpha, d, theta):
    """
    Denavit-Hartenberg transformation matrix
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics(joint_angles, dh_params):
    """
    Compute forward kinematics for a robot chain
    """
    T_total = np.eye(4)

    for i, (a, alpha, d, theta_offset) in enumerate(dh_params):
        theta = theta_offset + joint_angles[i]
        T_link = dh_transform(a, alpha, d, theta)
        T_total = T_total @ T_link

    return T_total
```

### Using Modern Libraries
```python
import roboticstoolbox as rtb
import spatialmath as sm

# Create a robot model
robot = rtb.DHRobot([
    rtb.RevoluteDH(a=0.1, alpha=np.pi/2, d=0.2, offset=0),
    rtb.RevoluteDH(a=0.2, alpha=0, d=0, offset=0),
    # Add more links as needed
])

# Compute forward kinematics
q = [0.1, 0.2, 0.3]  # Joint angles
T = robot.fkine(q)  # Forward kinematics
print(f"End-effector pose: {T}")
```

## Inverse Kinematics

### Analytical vs Numerical Solutions
- **Analytical**: Exact solutions for simple kinematic chains
- **Numerical**: Iterative solutions for complex robots

### Numerical IK Implementation
```python
import numpy as np
from scipy.optimize import minimize

def inverse_kinematics_numerical(robot, target_pose, q_init):
    """
    Solve inverse kinematics using numerical optimization
    """
    def objective(q):
        # Compute current pose
        current_pose = robot.fkine(q)

        # Calculate error
        pos_error = np.linalg.norm(target_pose.t - current_pose.t)
        rot_error = np.arccos(
            np.clip((np.trace(target_pose.R.T @ current_pose.R) - 1) / 2, -1, 1)
        )

        return pos_error + 0.1 * rot_error

    result = minimize(objective, q_init, method='BFGS')

    if result.success:
        return result.x
    else:
        raise ValueError("IK solution not found")
```

### Jacobian-based IK
```python
def jacobian_ik(robot, target_pose, q_init, max_iterations=100, tolerance=1e-4):
    """
    Solve IK using Jacobian transpose/pseudoinverse method
    """
    q = q_init.copy()

    for i in range(max_iterations):
        current_pose = robot.fkine(q)

        # Calculate error
        pos_error = target_pose.t - current_pose.t
        rot_error = (target_pose.R - current_pose.R).ravel()

        error = np.concatenate([pos_error, rot_error])

        if np.linalg.norm(error) < tolerance:
            break

        # Compute Jacobian
        J = robot.jacob0(q)

        # Update joint angles
        dq = np.linalg.pinv(J) @ error
        q = q + 0.1 * dq  # Learning rate

    return q
```

## Balance and Stability Control

### Zero Moment Point (ZMP)
The ZMP is crucial for bipedal stability:

```python
class BalanceController:
    def __init__(self, robot_mass, gravity=9.81):
        self.mass = robot_mass
        self.gravity = gravity
        self.kp = 10.0  # Proportional gain
        self.kd = 2.0   # Derivative gain

    def compute_zmp(self, com_pos, com_vel, com_acc):
        """
        Compute Zero Moment Point
        """
        zmp_x = com_pos[0] - (com_acc[0] * com_pos[2]) / self.gravity
        zmp_y = com_pos[1] - (com_acc[1] * com_pos[2]) / self.gravity

        return np.array([zmp_x, zmp_y, 0.0])

    def balance_control(self, desired_zmp, current_zmp, current_com, dt):
        """
        Balance control using ZMP feedback
        """
        zmp_error = desired_zmp[:2] - current_zmp[:2]

        # Compute desired COM acceleration
        com_acc_desired = self.gravity / current_com[2] * zmp_error

        return com_acc_desired
```

### Capture Point
The capture point indicates where to step to stop:

```python
def compute_capture_point(com_pos, com_vel, com_height):
    """
    Compute capture point for balance recovery
    """
    omega = np.sqrt(9.81 / com_height)
    capture_x = com_pos[0] + com_vel[0] / omega
    capture_y = com_pos[1] + com_vel[1] / omega

    return np.array([capture_x, capture_y])
```

## Walking Pattern Generation

### Inverted Pendulum Model
```python
class WalkingPatternGenerator:
    def __init__(self, step_height=0.05, step_length=0.3, step_time=1.0):
        self.step_height = step_height
        self.step_length = step_length
        self.step_time = step_time

    def generate_foot_trajectory(self, start_pos, goal_pos, t):
        """
        Generate smooth foot trajectory
        """
        # Linear interpolation for horizontal movement
        x = start_pos[0] + (goal_pos[0] - start_pos[0]) * t
        y = start_pos[1] + (goal_pos[1] - start_pos[1]) * t

        # Vertical movement (parabolic lift)
        if t < 0.5:
            z = self.step_height * 4 * t  # Lift up
        else:
            z = self.step_height * 4 * (1 - t)  # Lift down

        return np.array([x, y, z])

    def generate_com_trajectory(self, support_foot, swing_foot, t):
        """
        Generate Center of Mass trajectory following the inverted pendulum
        """
        # Simplified: CoM follows a smooth path between feet
        com_x = support_foot[0] + (swing_foot[0] - support_foot[0]) * t * 0.5
        com_y = support_foot[1] + (swing_foot[1] - support_foot[1]) * t * 0.5

        # Maintain constant height for stability
        com_z = 0.8  # Typical CoM height for humanoid

        return np.array([com_x, com_y, com_z])
```

## Whole-Body Control

### Task-Priority Framework
```python
class WholeBodyController:
    def __init__(self, robot):
        self.robot = robot
        self.tasks = []

    def add_task(self, task_name, jacobian, desired_velocity, priority=0):
        """
        Add a control task with priority
        """
        task = {
            'name': task_name,
            'J': jacobian,
            'v_des': desired_velocity,
            'priority': priority
        }
        self.tasks.append(task)

        # Sort tasks by priority (higher priority first)
        self.tasks.sort(key=lambda x: x['priority'], reverse=True)

    def compute_joint_velocities(self):
        """
        Compute joint velocities using task-priority framework
        """
        q_dot = np.zeros(self.robot.n)
        I = np.eye(self.robot.n)

        for task in self.tasks:
            J = task['J']
            v_des = task['v_des']

            # Compute null space projection
            J_pinv = np.linalg.pinv(J @ I)
            q_dot_task = J_pinv @ (v_des - J @ q_dot)

            # Update joint velocities
            q_dot = q_dot + q_dot_task

            # Update null space projector
            I = I - J_pinv @ J

        return q_dot
```

## ROS 2 Integration

### Joint Trajectory Control
```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Joint trajectory publisher
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Initialize kinematics
        self.kinematics = self.initialize_kinematics()

    def send_trajectory(self, joint_names, positions, times):
        """
        Send joint trajectory to robot
        """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        for pos, time in zip(positions, times):
            point = JointTrajectoryPoint()
            point.positions = pos
            point.time_from_start = Duration(sec=int(time), nanosec=int((time % 1) * 1e9))
            traj_msg.points.append(point)

        self.joint_pub.publish(traj_msg)

    def move_to_pose(self, target_pose, end_effector_name):
        """
        Move end effector to target pose using IK
        """
        # Solve inverse kinematics
        joint_angles = self.inverse_kinematics(target_pose, end_effector_name)

        # Send to robot
        joint_names = self.get_joint_names(end_effector_name)
        self.send_trajectory([joint_names], [joint_angles], [2.0])  # 2 second move
```

## Walking Control Implementation

### Walking State Machine
```python
from enum import Enum

class WalkingState(Enum):
    STANDING = 1
    LEFT_SWING = 2
    RIGHT_SWING = 3
    DOUBLE_SUPPORT = 4

class WalkingController:
    def __init__(self):
        self.state = WalkingState.STANDING
        self.left_foot_support = True
        self.cycle_time = 1.0
        self.current_time = 0.0
        self.step_count = 0

    def update(self, dt):
        """
        Update walking state machine
        """
        self.current_time += dt

        if self.current_time >= self.cycle_time:
            self.current_time = 0.0
            self.toggle_support_foot()
            self.step_count += 1

        # Generate appropriate trajectories based on state
        left_foot_pos = self.generate_foot_trajectory('left', self.current_time / self.cycle_time)
        right_foot_pos = self.generate_foot_trajectory('right', self.current_time / self.cycle_time)

        return left_foot_pos, right_foot_pos

    def generate_foot_trajectory(self, foot, phase):
        """
        Generate foot trajectory based on walking phase
        """
        if foot == 'left' and self.left_foot_support:
            # Left foot is support foot - stay in place
            return self.get_support_foot_position()
        elif foot == 'right' and not self.left_foot_support:
            # Right foot is support foot - stay in place
            return self.get_support_foot_position()
        else:
            # Swing foot trajectory
            return self.get_swing_foot_trajectory(phase)

    def toggle_support_foot(self):
        """
        Toggle which foot is the support foot
        """
        self.left_foot_support = not self.left_foot_support
```

## Balance Recovery Strategies

### Push Recovery
```python
class BalanceRecovery:
    def __init__(self, robot_controller):
        self.controller = robot_controller
        self.critical_angle = np.radians(15)  # 15 degrees

    def detect_imminent_fall(self, imu_data, com_state):
        """
        Detect if robot is about to fall
        """
        roll, pitch = imu_data['orientation'][:2]

        if abs(roll) > self.critical_angle or abs(pitch) > self.critical_angle:
            return True

        # Check if CoM is outside support polygon
        if self.is_com_outside_support(com_state):
            return True

        return False

    def execute_recovery(self, recovery_strategy='step'):
        """
        Execute balance recovery based on strategy
        """
        if recovery_strategy == 'step':
            self.take_recovery_step()
        elif recovery_strategy == 'crouch':
            self.crouch_down()
        elif recovery_strategy == 'arm_swing':
            self.swing_arms_for_balance()

    def take_recovery_step(self):
        """
        Take a step to recover balance
        """
        # Compute capture point
        capture_point = self.compute_capture_point()

        # Take step toward capture point
        self.controller.step_to(capture_point)
```

## Simulation and Validation

### Gazebo Integration
```xml
<!-- Humanoid robot control plugin -->
<gazebo>
  <plugin name="humanoid_control_plugin" filename="libhumanoid_control.so">
    <robot_namespace>/humanoid</robot_namespace>
    <joints>
      <joint>left_hip_roll</joint>
      <joint>left_hip_yaw</joint>
      <joint>left_hip_pitch</joint>
      <joint>left_knee</joint>
      <joint>left_ankle_pitch</joint>
      <joint>left_ankle_roll</joint>
      <!-- Add all other joints -->
    </joints>
    <control_mode>position</control_mode>
    <gains>
      <left_hip_roll_p>100.0</left_hip_roll_p>
      <left_hip_roll_i>0.1</left_hip_roll_i>
      <left_hip_roll_d>10.0</left_hip_roll_d>
    </gains>
  </plugin>
</gazebo>
```

## Performance Considerations

### Real-time Requirements
- **Control Rate**: 100-1000 Hz for stable control
- **Computation Time**: Keep IK solutions under 10ms
- **Communication**: Low-latency joint control

### Optimization Techniques
```python
# Use analytical solutions when possible
def analytical_ik_6dof(wrist_position, wrist_orientation):
    """
    Analytical solution for 6-DOF manipulator
    Much faster than numerical methods
    """
    # Implementation of closed-form solution
    pass

# Cache transformations
class CachedKinematics:
    def __init__(self):
        self.transform_cache = {}
        self.last_config = None

    def get_transform(self, q, link_name):
        """
        Get transform with caching to improve performance
        """
        cache_key = (tuple(np.round(q, 3)), link_name)

        if cache_key != self.last_config:
            self.transform_cache[cache_key] = self.compute_transform(q, link_name)
            self.last_config = cache_key

        return self.transform_cache[cache_key]
```

## Best Practices

1. **Stability First**: Prioritize balance over other objectives
2. **Smooth Transitions**: Ensure smooth transitions between behaviors
3. **Safety Limits**: Respect joint limits and actuator constraints
4. **Modular Design**: Separate kinematics, dynamics, and control
5. **Validation**: Test extensively in simulation before real robot

## Learning Objectives

After completing this module, you will be able to:
- Implement forward and inverse kinematics for humanoid robots
- Design balance control systems using ZMP and capture point
- Generate walking patterns for bipedal locomotion
- Implement whole-body control with task prioritization
- Integrate kinematic control with ROS 2
- Handle balance recovery and push resistance