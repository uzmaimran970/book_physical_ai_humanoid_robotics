---
id: module2-chapter2
title: Gazebo Simulation
sidebar_label: Chapter 2 - Gazebo
---

# Gazebo Simulation

## Introduction to Gazebo

Gazebo is the most widely used robot simulator in the ROS ecosystem, providing accurate physics simulation, realistic sensor models, and seamless ROS 2 integration.

### Gazebo Variants

- **Gazebo Classic** (v11): Mature, stable, ROS 1 & ROS 2 compatible
- **Gazebo Sim** (Ignition): Modern architecture, better performance
- **Gazebo Harmonic**: Latest unified release

This chapter focuses on Gazebo with ROS 2 integration.

## Installation and Setup

### Install Gazebo with ROS 2

```bash
# Ubuntu 22.04 + ROS 2 Humble
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify installation
gazebo --version

# Test Gazebo with ROS 2
ros2 launch gazebo_ros gazebo.launch.py
```

### Workspace Setup

```bash
# Create workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone example packages
git clone https://github.com/ros/gazebo_ros_demos.git

# Build workspace
cd ~/robot_ws
colcon build
source install/setup.bash
```

## Creating Your First Robot in Gazebo

### Project Structure

```
my_robot/
├── models/
│   └── my_robot/
│       ├── model.config
│       ├── model.sdf
│       └── meshes/
├── worlds/
│   └── my_world.world
├── urdf/
│   └── my_robot.urdf.xacro
└── launch/
    └── simulation.launch.py
```

### Simple Mobile Robot URDF

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Properties -->
  <xacro:property name="base_width" value="0.4"/>
  <xacro:property name="base_length" value="0.6"/>
  <xacro:property name="base_height" value="0.2"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15"/>
      <inertia ixx="0.32" ixy="0" ixz="0"
               iyy="0.52" iyz="0" izz="0.72"/>
    </inertial>
  </link>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="prefix y_reflect">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="1.57 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="1.57 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0" ixz="0"
                 iyy="0.001" iyz="0" izz="0.002"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0.2 ${y_reflect * base_width/2 + y_reflect * wheel_width/2} -${base_height/2}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="left" y_reflect="1"/>
  <xacro:wheel prefix="right" y_reflect="-1"/>

  <!-- Caster wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.25 0 -0.15" rpy="0 0 0"/>
  </joint>

</robot>
```

### Adding Gazebo-Specific Tags

```xml
<!-- Add to URDF after closing </robot> tag -->
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
</gazebo>

<gazebo reference="left_wheel">
  <material>Gazebo/Black</material>
  <mu1>1.0</mu1>  <!-- Friction coefficient -->
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>  <!-- Stiffness -->
  <kd>100.0</kd>  <!-- Damping -->
</gazebo>

<gazebo reference="right_wheel">
  <material>Gazebo/Black</material>
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

## Gazebo Plugins

Plugins add functionality to simulated robots.

### Differential Drive Plugin

```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <!-- Wheel joints -->
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>

    <!-- Kinematics -->
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>

    <!-- Limits -->
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>

    <!-- Output -->
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>

    <!-- Topics -->
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>

    <!-- Update rate -->
    <update_rate>50</update_rate>
  </plugin>
</gazebo>
```

### Common Gazebo Plugins

#### 1. Camera Plugin

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>robot/camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

#### 2. Lidar Plugin

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

#### 3. IMU Plugin

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

## Creating Simulation Worlds

### Basic World File

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Gazebo GUI -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-5 -5 5 0 0.5 0.8</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
  </world>
</sdf>
```

## Launch File for Gazebo Simulation

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare('my_robot').find('my_robot')

    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.world')

    # Process xacro to URDF
    robot_description_command = f'xacro {urdf_file}'

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
```

## Testing Your Simulation

### Launch Simulation

```bash
# Source workspace
source ~/robot_ws/install/setup.bash

# Launch simulation
ros2 launch my_robot simulation.launch.py

# In another terminal, test velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"

# Monitor odometry
ros2 topic echo /odom

# Visualize in RViz
ros2 run rviz2 rviz2
```

### Debugging Common Issues

#### Robot Falls Through Ground
```xml
<!-- Increase collision parameters -->
<gazebo reference="base_link">
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
</gazebo>
```

#### Robot Moves Erratically
```xml
<!-- Check inertia values -->
<inertial>
  <mass value="10"/>
  <!-- Use realistic inertia calculator -->
  <inertia ixx="0.32" ixy="0" ixz="0"
           iyy="0.52" iyz="0" izz="0.72"/>
</inertial>
```

#### Poor Performance
```xml
<!-- Reduce physics update rate -->
<physics type="ode">
  <max_step_size>0.01</max_step_size>  <!-- Increased from 0.001 -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- Decreased from 1000 -->
</physics>
```

## Advanced Topics

### 1. Programmatic World Generation

```python
# Generate world with obstacles
import random
from jinja2 import Template

template = Template('''
<sdf version="1.6">
  <world name="random_world">
    <include><uri>model://ground_plane</uri></include>
    {% for i in range(num_obstacles) %}
    <model name="box_{{ i }}">
      <pose>{{ x[i] }} {{ y[i] }} 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 1 1</size></box></geometry>
        </visual>
      </link>
    </model>
    {% endfor %}
  </world>
</sdf>
''')

# Generate random obstacles
num_obstacles = 10
x = [random.uniform(-10, 10) for _ in range(num_obstacles)]
y = [random.uniform(-10, 10) for _ in range(num_obstacles)]

world_sdf = template.render(num_obstacles=num_obstacles, x=x, y=y)
with open('random_world.world', 'w') as f:
    f.write(world_sdf)
```

### 2. Sensor Data Recording

```bash
# Record simulation data
ros2 bag record /scan /camera/image_raw /odom /cmd_vel

# Replay for analysis
ros2 bag play my_simulation.db3
```

### 3. Automated Testing

```python
# Test navigation in simulation
class SimulationTester:
    def __init__(self):
        self.start_gazebo()
        self.spawn_robot()

    def test_obstacle_avoidance(self):
        # Send robot toward obstacle
        self.publish_cmd_vel(0.5, 0)

        # Wait and check if robot stopped
        time.sleep(5)
        odom = self.get_odometry()

        assert odom.twist.linear.x < 0.1, "Robot should have stopped"

    def cleanup(self):
        self.shutdown_gazebo()
```

## Key Takeaways

- Gazebo provides **accurate physics** and **ROS 2 integration**
- **URDF/Xacro** define robot structure and properties
- **Plugins** add sensors and actuators to simulations
- **World files** define environments and obstacles
- **Launch files** coordinate simulation startup
- **Testing** in simulation accelerates development

## Next Steps

In the next chapter, you'll learn about:
- Unity Robotics integration
- Photorealistic rendering for computer vision
- ML-Agents for reinforcement learning
- Synthetic data generation
