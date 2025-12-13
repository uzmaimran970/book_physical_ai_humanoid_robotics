---
id: module3-chapter2
title: Isaac Navigation Stack
sidebar_label: Chapter 2 - Navigation
---

# Isaac Navigation Stack

## Nav2 Integration with Isaac Sim

Navigation 2 (Nav2) is the ROS 2 navigation framework. Isaac Sim provides seamless integration for autonomous mobile robot navigation.

### Navigation Stack Overview

```
Sensors (Lidar, Camera, IMU)
          ↓
    Localization (AMCL)
          ↓
    Costmap (Global + Local)
          ↓
    Planner (NavFn, Smac)
          ↓
    Controller (DWB, TEB)
          ↓
    Velocity Commands
```

## Setting Up Navigation

### Install Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Map Creation with SLAM

```python
# Isaac Sim: Create environment
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid

world = World()

# Create obstacles
for i in range(10):
    obstacle = VisualCuboid(
        prim_path=f"/World/Obstacle_{i}",
        position=[random.uniform(-5, 5), random.uniform(-5, 5), 0.5],
        scale=[0.5, 0.5, 1.0]
    )

# Add robot with lidar
robot = world.scene.add(
    Robot(prim_path="/World/Robot", name="robot")
)
```

### SLAM Toolbox

```bash
# Launch SLAM
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true

# Save map
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Navigation Configuration

### Navigation Parameters

```yaml
# nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

### Launch Navigation

```python
# navigation.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    return LaunchDescription([
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            parameters=[nav2_params]
        ),

        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[nav2_params]
        ),

        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[nav2_params]
        ),

        # Behavior server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            parameters=[nav2_params]
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[nav2_params]
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),
    ])
```

## Path Planning Algorithms

### Global Planners

**1. NavFn (Dijkstra's)**
- Fast, guaranteed optimal
- Grid-based
- Good for static environments

**2. Smac Planner**
- State lattice-based
- Considers robot kinematics
- Better for constrained spaces

**3. Theta* Planner**
- Any-angle paths
- Smoother than grid-based
- Reduced path length

### Local Planners

**1. DWB (Dynamic Window Approach)**
- Velocity space planning
- Real-time obstacle avoidance
- Configurable trajectory scoring

**2. TEB (Timed Elastic Band)**
- Optimizes trajectory in time
- Smooth motion
- Better for dynamic environments

## Behavior Trees

Nav2 uses behavior trees for decision-making.

### Custom Navigation Behavior

```xml
<!-- custom_bt.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <RecoveryNode number_of_retries="6" name="RecoveryWithFeedback">
          <PipelineSequence name="NavigateWithFeedback">
            <ComputePathToPose goal="{goal}" path="{path}"/>
            <FollowPath path="{path}"/>
          </PipelineSequence>
          <SequenceStar name="RecoveryActions">
            <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
            <Spin spin_dist="1.57"/>
            <Wait wait_duration="5"/>
          </SequenceStar>
        </RecoveryNode>
      </RateController>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

## Obstacle Avoidance

### Costmap Layers

```yaml
local_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_link
    rolling_window: true
    width: 3
    height: 3
    resolution: 0.05
    plugins: ["voxel_layer", "inflation_layer"]

    voxel_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
      enabled: True
      publish_voxel_map: True
      origin_z: 0.0
      z_resolution: 0.05
      z_voxels: 16
      max_obstacle_height: 2.0
      mark_threshold: 0
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.55
```

## Multi-Robot Navigation

```python
# Multi-robot coordination
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class FleetManager:
    def __init__(self, num_robots=3):
        self.navigators = [
            BasicNavigator(namespace=f'robot{i}')
            for i in range(num_robots)
        ]

    def navigate_fleet(self, goals):
        # Send goals to all robots
        for nav, goal in zip(self.navigators, goals):
            nav.goToPose(goal)

        # Wait for all to complete
        while not all(nav.isTaskComplete() for nav in self.navigators):
            rclpy.spin_once(self.navigators[0], timeout_sec=0.1)

        print("Fleet navigation complete!")
```

## Key Takeaways

- **Nav2** provides complete autonomous navigation
- **SLAM** creates maps for localization
- **Path planning** algorithms vary in optimality and speed
- **Behavior trees** enable flexible navigation logic
- **Multi-robot** coordination requires namespace management

## Practical Exercises

1. Create navigation stack for Isaac Sim robot
2. Tune costmap parameters for obstacle avoidance
3. Implement custom behavior tree
4. Coordinate 3 robots in warehouse simulation

## Next Steps

In the next chapter:
- Perception and manipulation
- Object detection and grasping
- Isaac Manipulator workflows
- AI-accelerated pick-and-place
