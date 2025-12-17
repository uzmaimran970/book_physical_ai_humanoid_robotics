---
id: module1-chapter3
title: ROS 2 Launch Files and Parameters
sidebar_label: Chapter 3 - Launch & Parameters
---

# ROS 2 Launch Files and Parameters

## Introduction

Real robotics applications require coordinating multiple nodes, configuring parameters, and managing complex startup sequences. **Launch files** automate this process, while **parameters** provide runtime configuration without code changes.

## ROS 2 Launch System

Launch files orchestrate the startup of multiple nodes and configure the ROS 2 environment.

### Why Use Launch Files?

Without launch files, you would need to:
```bash
# Terminal 1
ros2 run pkg1 node1

# Terminal 2
ros2 run pkg2 node2

# Terminal 3
ros2 run pkg3 node3
```

With launch files, one command starts everything:
```bash
ros2 launch my_robot robot.launch.py
```

### Launch File Formats

ROS 2 supports three formats:

1. **Python** (`.launch.py`) - Most flexible, recommended
2. **XML** (`.launch.xml`) - Similar to ROS 1
3. **YAML** (`.launch.yaml`) - Simple configurations

## Python Launch Files

Python launch files offer the most flexibility and are the recommended format.

### Basic Launch File Structure

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener'
        ),
    ])
```

### Launch File Components

#### 1. Node Actions

Start ROS 2 nodes:

```python
Node(
    package='my_robot_pkg',
    executable='controller',
    name='robot_controller',
    namespace='robot1',
    output='screen',  # Show node output
    parameters=[{'use_sim_time': True}],
    remappings=[('/cmd_vel', '/robot1/cmd_vel')],
    arguments=['--ros-args', '--log-level', 'INFO']
)
```

#### 2. Include Actions

Include other launch files:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_pkg')
    sensors_launch = os.path.join(pkg_dir, 'launch', 'sensors.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch),
            launch_arguments={'robot_name': 'robot1'}.items()
        ),
    ])
```

#### 3. Conditional Execution

Launch nodes conditionally:

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity',
            condition=IfCondition(use_sim)
        ),
    ])
```

## Complete Robot Launch Example

Here's a comprehensive launch file for a mobile robot:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('my_robot').find('my_robot')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(os.path.join(pkg_share, 'urdf', 'robot.urdf')).read()
        }]
    )

    # Controller node
    controller = Node(
        package='my_robot',
        executable='controller',
        name='controller',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_speed': 1.0,
            'control_frequency': 20.0
        }],
        remappings=[
            ('/cmd_vel', [robot_name, '/cmd_vel']),
            ('/odom', [robot_name, '/odom'])
        ]
    )

    # Sensor drivers
    lidar_driver = Node(
        package='my_robot',
        executable='lidar_driver',
        name='lidar_driver',
        namespace=robot_name,
        parameters=[{
            'frame_id': [robot_name, '/laser_frame'],
            'scan_frequency': 10.0
        }]
    )

    camera_driver = Node(
        package='my_robot',
        executable='camera_driver',
        name='camera_driver',
        namespace=robot_name,
        parameters=[{
            'frame_id': [robot_name, '/camera_frame'],
            'image_width': 640,
            'image_height': 480,
            'fps': 30
        }]
    )

    # RViz (conditional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot.rviz')],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_robot_name,
        declare_use_rviz,

        # Launch nodes
        robot_state_publisher,
        controller,
        lidar_driver,
        camera_driver,
        rviz,
    ])
```

Run this launch file with:
```bash
ros2 launch my_robot robot.launch.py robot_name:=robot1 use_rviz:=true
```

## ROS 2 Parameters

Parameters provide runtime configuration for nodes without recompiling code.

### Parameter Types

ROS 2 supports these parameter types:
- `bool`: True/False
- `int`: Integer values
- `float`: Floating-point values
- `string`: Text strings
- `byte_array`: Binary data
- `bool_array`, `int_array`, `float_array`, `string_array`: Arrays

### Declaring Parameters in Nodes

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensors', ['lidar', 'camera'])
        self.declare_parameter('debug_mode', False)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.sensors = self.get_parameter('sensors').value
        self.debug = self.get_parameter('debug_mode').value

        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max speed: {self.max_speed}')
        self.get_logger().info(f'Sensors: {self.sensors}')
```

### Parameter Callbacks

React to parameter changes at runtime:

```python
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class DynamicNode(Node):
    def __init__(self):
        super().__init__('dynamic_node')

        # Declare parameter with descriptor
        descriptor = ParameterDescriptor(
            description='Maximum velocity in m/s',
            additional_constraints='Must be between 0.0 and 2.0'
        )
        self.declare_parameter('max_velocity', 1.0, descriptor)

        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity':
                if 0.0 <= param.value <= 2.0:
                    self.get_logger().info(
                        f'Max velocity updated to {param.value}')
                    return SetParametersResult(successful=True)
                else:
                    self.get_logger().warn(
                        f'Invalid max_velocity: {param.value}')
                    return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)
```

### Parameter Files (YAML)

Store parameters in YAML files:

```yaml
# robot_params.yaml
/**:
  ros__parameters:
    robot_name: "robot1"
    max_speed: 1.5
    control_frequency: 20.0
    sensors:
      - lidar
      - camera
      - imu
    debug_mode: false

/controller:
  ros__parameters:
    pid_gains:
      kp: 1.0
      ki: 0.1
      kd: 0.01
    max_acceleration: 0.5
```

Load parameters in launch file:

```python
controller = Node(
    package='my_robot',
    executable='controller',
    parameters=[os.path.join(pkg_share, 'config', 'robot_params.yaml')]
)
```

### Command-Line Parameter Operations

```bash
# List node parameters
ros2 param list /controller

# Get parameter value
ros2 param get /controller max_speed

# Set parameter value
ros2 param set /controller max_speed 2.0

# Dump all parameters to file
ros2 param dump /controller > controller_params.yaml

# Load parameters from file
ros2 param load /controller controller_params.yaml
```

## Advanced Launch Techniques

### 1. Event Handlers

Execute actions based on events:

```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    talker = Node(package='demo_nodes_cpp', executable='talker')

    listener = Node(package='demo_nodes_cpp', executable='listener')

    # Start listener when talker starts
    register_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=talker,
            on_start=[listener]
        )
    )

    return LaunchDescription([talker, register_event_handler])
```

### 2. Composable Nodes

Load multiple nodes into a single process for efficiency:

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_robot',
                plugin='my_robot::Controller',
                name='controller'
            ),
            ComposableNode(
                package='my_robot',
                plugin='my_robot::Sensors',
                name='sensors'
            ),
        ],
    )

    return LaunchDescription([container])
```

### 3. Multi-Robot Systems

Launch multiple robots with namespacing:

```python
def generate_launch_description():
    robots = []

    for i in range(3):
        robot_name = f'robot{i}'

        robot_nodes = [
            Node(
                package='my_robot',
                executable='controller',
                namespace=robot_name,
                parameters=[{'robot_id': i}]
            ),
            Node(
                package='my_robot',
                executable='sensors',
                namespace=robot_name,
                parameters=[{'robot_id': i}]
            ),
        ]

        robots.extend(robot_nodes)

    return LaunchDescription(robots)
```

## Module 1 Summary

### Key Concepts

#### Chapter 1: Nodes and Topics
- ✅ Nodes are independent processes performing specific tasks
- ✅ Topics enable publish-subscribe communication
- ✅ Messages are typed data structures
- ✅ QoS policies control message delivery

#### Chapter 2: Services and Actions
- ✅ Services provide request-response communication
- ✅ Actions handle long-running tasks with feedback
- ✅ Choose communication pattern based on requirements
- ✅ Implement proper error handling and timeouts

#### Chapter 3: Launch Files and Parameters
- ✅ Launch files orchestrate multi-node systems
- ✅ Parameters enable runtime configuration
- ✅ Use YAML files for parameter organization
- ✅ Event handlers coordinate node startup

### Practical Exercises

1. **Build a Multi-Sensor System**
   - Create nodes for lidar, camera, IMU
   - Use topics for sensor data streaming
   - Add service for sensor calibration
   - Write launch file to start all sensors

2. **Implement Robot Controller**
   - Parameter-based velocity limits
   - Service for changing modes (manual/auto)
   - Action for navigation tasks
   - Launch file with configuration

3. **Multi-Robot Coordination**
   - Launch 3 robots with unique namespaces
   - Shared parameter file
   - Topic remapping for coordination
   - Conditional visualization

### Visual Diagrams (Conceptual)

```
ROS 2 System Architecture:

┌─────────────────────────────────────────────┐
│  Launch File (robot.launch.py)               │
│  ┌─────────┐  ┌──────────┐  ┌────────────┐  │
│  │ Node 1  │  │  Node 2  │  │   Node 3   │  │
│  │ Sensors │  │Controller│  │ Navigation │  │
│  └────┬────┘  └─────┬────┘  └──────┬─────┘  │
│       │             │              │        │
│  ┌────▼─────────────▼──────────────▼─────┐  │
│  │          Parameter Server            │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │     Topics / Services / Actions       │  │
│  └──────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

### Resources for Further Learning

- **Official ROS 2 Documentation**: docs.ros.org
- **ROS 2 Tutorials**: index.ros.org/doc/ros2/Tutorials
- **ROS 2 Design**: design.ros2.org
- **ROS Discourse**: discourse.ros.org

### Next Module Preview

**Module 2: The Digital Twin (Gazebo & Unity)**
- Simulation environments for robot testing
- Creating accurate robot models (URDF/SDF)
- Physics simulation and sensor modeling
- Sim-to-real transfer techniques
