export const bookData = {
  title: "Physical AI & Humanoid Robotics",
  subtitle: "A Comprehensive Guide to Modern Robotics",
  description: "Master the fundamentals of robotics, from ROS 2 to Vision-Language-Action models",
  modules: [
    {
      id: "module-1",
      number: 1,
      title: "The Robotic Nervous System (ROS 2)",
      description: "Learn the foundational communication framework for modern robotics",
      chapters: [
        {
          id: "m1-c1",
          moduleId: "module-1",
          number: 1,
          title: "ROS 2 Nodes and Topics",
          content: `
# ROS 2 Nodes and Topics

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is the successor to ROS 1, designed to meet the demands of production robotics systems. It provides a distributed architecture where different processes (nodes) can communicate with each other.

## Understanding Nodes

A **node** is a process that performs computation. In ROS 2, each node is responsible for a single, modular purpose:

- A node for controlling wheel motors
- A node for processing laser scanner data
- A node for planning paths

### Creating Your First Node

\`\`\`python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

## Topics: The Communication Highways

Topics are named buses over which nodes exchange messages. They implement a **publish-subscribe** pattern:

- **Publishers** send data to a topic
- **Subscribers** receive data from a topic

### Publisher Example

\`\`\`python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Robot is operational'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
\`\`\`

### Subscriber Example

\`\`\`python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
\`\`\`

## Message Types

ROS 2 uses strongly-typed messages. Common types include:

- \`std_msgs/String\` - Text messages
- \`std_msgs/Int32\` - Integer values
- \`sensor_msgs/Image\` - Camera images
- \`geometry_msgs/Twist\` - Velocity commands

## Quality of Service (QoS)

ROS 2 introduces QoS policies for reliable communication:

\`\`\`python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
\`\`\`

## Practice Exercises

1. **Exercise 1**: Create a node that publishes temperature sensor data every 2 seconds
2. **Exercise 2**: Create a subscriber that receives temperature data and logs warnings if it exceeds 80°C
3. **Exercise 3**: Build a simple "robot heartbeat" system with publisher and subscriber nodes

## Key Takeaways

✅ Nodes are independent processes that perform specific tasks
✅ Topics enable many-to-many communication between nodes
✅ Publishers send data, subscribers receive data
✅ QoS policies ensure reliable message delivery
✅ Message types provide structure and type safety
`
        },
        {
          id: "m1-c2",
          moduleId: "module-1",
          number: 2,
          title: "ROS 2 Services and Actions",
          content: `
# ROS 2 Services and Actions

## Services: Request-Response Communication

While topics are great for streaming data, sometimes you need a **request-response** pattern. Services provide synchronous, two-way communication.

### When to Use Services

- Trigger a specific behavior (e.g., "take a photo now")
- Query the robot's state
- Perform calculations
- Reset configurations

### Creating a Service Server

\`\`\`python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
\`\`\`

### Creating a Service Client

\`\`\`python
class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client.call_async(request)
        return future
\`\`\`

## Actions: Long-Running Tasks

Actions are designed for tasks that:
- Take time to complete
- Provide feedback during execution
- Can be cancelled

### Action Structure

Actions have three components:

1. **Goal** - What you want to achieve
2. **Feedback** - Progress updates
3. **Result** - Final outcome

### Example: Navigation Action

\`\`\`python
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    async def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            # Send feedback
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
\`\`\`

### Action Client

\`\`\`python
class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received: {feedback_msg.feedback.sequence}')
\`\`\`

## Comparison: Topics vs Services vs Actions

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| Pattern | Pub/Sub | Request/Reply | Goal/Feedback/Result |
| Blocking | No | Yes | No (async) |
| Feedback | No | No | Yes |
| Cancellable | N/A | No | Yes |
| Use Case | Streaming | One-off requests | Long tasks |

## Practice Exercises

1. **Exercise 1**: Create a service that converts temperature from Celsius to Fahrenheit
2. **Exercise 2**: Build an action server that simulates a robot charging battery (0-100%)
3. **Exercise 3**: Implement a "patrol" action that visits multiple waypoints with feedback

## Key Takeaways

✅ Services provide synchronous request-response communication
✅ Actions are ideal for long-running, cancellable tasks
✅ Actions provide continuous feedback during execution
✅ Choose the right pattern based on your communication needs
`
        },
        {
          id: "m1-c3",
          moduleId: "module-1",
          number: 3,
          title: "ROS 2 Launch Files and Parameters",
          content: `
# ROS 2 Launch Files and Parameters

## Introduction to Launch Files

Launch files allow you to start multiple nodes with a single command. They're essential for complex robotic systems.

### Why Use Launch Files?

- Start multiple nodes simultaneously
- Set parameters for each node
- Organize system configuration
- Enable/disable nodes conditionally
- Remap topics and services

## Python Launch Files

ROS 2 uses Python for launch files, providing programming flexibility:

\`\`\`python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='sensor_node',
            name='lidar',
            output='screen'
        ),
        Node(
            package='my_robot',
            executable='motor_controller',
            name='motors',
            output='screen'
        ),
    ])
\`\`\`

### Advanced Launch Features

\`\`\`python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation mode'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity',
            condition=IfCondition(use_sim),
            arguments=['-entity', 'robot', '-file', 'robot.urdf']
        ),
    ])
\`\`\`

## Parameters: Configuring Nodes

Parameters allow dynamic configuration without recompiling code.

### Declaring Parameters

\`\`\`python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('robot_name', 'my_robot')

        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.robot_name = self.get_parameter('robot_name').value

        self.get_logger().info(f'Max speed: {self.max_speed}')
        self.get_logger().info(f'Robot name: {self.robot_name}')
\`\`\`

### Setting Parameters in Launch Files

\`\`\`python
Node(
    package='my_robot',
    executable='driver_node',
    parameters=[{
        'max_speed': 2.5,
        'robot_name': 'atlas',
        'use_lidar': True
    }]
)
\`\`\`

### Loading Parameters from YAML

**config/params.yaml:**
\`\`\`yaml
robot_driver:
  ros__parameters:
    max_speed: 2.5
    min_speed: 0.1
    acceleration: 1.0
    wheel_radius: 0.05
\`\`\`

**Launch file:**
\`\`\`python
import os
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
    get_package_share_directory('my_robot'),
    'config',
    'params.yaml'
)

Node(
    package='my_robot',
    executable='driver_node',
    parameters=[config]
)
\`\`\`

## Remapping Topics

Remapping allows you to change topic names without modifying code:

\`\`\`python
Node(
    package='my_robot',
    executable='camera_node',
    remappings=[
        ('/camera/image', '/front_camera/image'),
        ('/camera/info', '/front_camera/info')
    ]
)
\`\`\`

## Multi-Robot Systems

Launch files excel at managing multi-robot setups:

\`\`\`python
def generate_launch_description():
    robots = []

    for i in range(3):
        robot_node = Node(
            package='my_robot',
            executable='robot_controller',
            name=f'robot_{i}',
            namespace=f'robot{i}',
            parameters=[{
                'robot_id': i,
                'x_pos': i * 2.0,
                'y_pos': 0.0
            }]
        )
        robots.append(robot_node)

    return LaunchDescription(robots)
\`\`\`

## Practice Exercises

1. **Exercise 1**: Create a launch file that starts a sensor node and a processing node
2. **Exercise 2**: Add parameters to configure sensor update rate and processing threshold
3. **Exercise 3**: Build a multi-robot launch file with 5 robots in different namespaces

## Key Takeaways

✅ Launch files simplify starting complex robotic systems
✅ Parameters enable runtime configuration
✅ YAML files organize parameters cleanly
✅ Remapping provides flexibility in topic naming
✅ Namespaces enable multi-robot systems
`
        }
      ]
    },
    {
      id: "module-2",
      number: 2,
      title: "The Digital Twin (Gazebo & Unity)",
      description: "Build realistic simulations for testing and training robots",
      chapters: [
        {
          id: "m2-c1",
          moduleId: "module-2",
          number: 1,
          title: "Introduction to Digital Twins",
          content: `
# Introduction to Digital Twins

## What is a Digital Twin?

A **digital twin** is a virtual representation of a physical robot that mirrors its behavior, environment, and interactions in real-time or simulated time.

## Benefits of Digital Twins

### 1. Safe Testing Environment
- Test dangerous scenarios without risk
- No hardware damage during development
- Unlimited experimentation

### 2. Cost Efficiency
- Develop algorithms before hardware arrives
- Reduce wear and tear on physical robots
- Test multiple scenarios simultaneously

### 3. Accelerated Development
- Faster iteration cycles
- Parallel testing across configurations
- Easy scenario reproduction

## Simulation Platforms Comparison

| Feature | Gazebo | Unity | Isaac Sim |
|---------|--------|-------|-----------|
| Physics | ODE, Bullet | PhysX | PhysX (GPU) |
| Graphics | Modest | Excellent | Photorealistic |
| ROS Integration | Native | Plugin | Native |
| ML/AI | Limited | ML-Agents | Advanced |
| Cost | Free | Free tier | Free |
| Best For | Traditional robotics | Game-like environments | AI/ML training |

## URDF: Universal Robot Description Format

URDF is XML-based format for describing robot structure:

\`\`\`xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Wheel link -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting base to wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.25 0" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
\`\`\`

## SDF: Simulation Description Format

SDF extends URDF with simulation-specific features:

\`\`\`xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="advanced_robot">
    <pose>0 0 0.5 0 0 0</pose>

    <link name="chassis">
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>1920</width>
            <height>1080</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <update_rate>30</update_rate>
      </sensor>

      <sensor name="lidar" type="ray">
        <ray>
          <scan>
            <horizontal>
              <samples>640</samples>
              <resolution>1</resolution>
              <min_angle>-1.57</min_angle>
              <max_angle>1.57</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>30.0</max>
          </range>
        </ray>
        <update_rate>10</update_rate>
      </sensor>
    </link>
  </model>
</sdf>
\`\`\`

## Physics Engines

### ODE (Open Dynamics Engine)
- Fast, stable
- Good for wheeled robots
- Default in Gazebo Classic

### Bullet
- Accurate collision detection
- Soft body physics
- Good for manipulation

### PhysX
- GPU-accelerated
- High performance
- Used in Unity and Isaac Sim

## Practice Exercises

1. **Exercise 1**: Create a URDF file for a simple 2-wheeled robot
2. **Exercise 2**: Add a camera sensor to your robot using SDF
3. **Exercise 3**: Compare physics engines by dropping objects from height

## Key Takeaways

✅ Digital twins enable safe, cost-effective robot development
✅ URDF describes robot structure and appearance
✅ SDF adds simulation-specific features
✅ Choose simulation platform based on your needs
✅ Physics engines affect simulation accuracy and performance
`
        },
        {
          id: "m2-c2",
          moduleId: "module-2",
          number: 2,
          title: "Gazebo Simulation",
          content: `
# Gazebo Simulation

## Introduction to Gazebo

Gazebo is the most popular robotics simulator, tightly integrated with ROS. It provides realistic physics, sensors, and environments.

## Installing Gazebo with ROS 2

\`\`\`bash
# Install Gazebo Garden (recommended for ROS 2)
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install gazebo

# Verify installation
gazebo --version
\`\`\`

## Launching Gazebo

\`\`\`bash
# Launch empty world
gazebo

# Launch with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py
\`\`\`

## Creating Custom Worlds

### Simple World File

\`\`\`xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="robot_world">
    <!-- Physics -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include models -->
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
\`\`\`

## Spawning Robots in Gazebo

### Using Launch Files

\`\`\`python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                        'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': 'robot_world.sdf'}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-file', 'robot.urdf',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot
    ])
\`\`\`

## Gazebo Plugins

Plugins extend Gazebo's functionality:

### Differential Drive Plugin

\`\`\`xml
<plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
  <update_rate>100</update_rate>

  <!-- Wheels -->
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>

  <!-- Kinematics -->
  <wheel_separation>0.5</wheel_separation>
  <wheel_diameter>0.2</wheel_diameter>

  <!-- Limits -->
  <max_wheel_torque>20</max_wheel_torque>
  <max_wheel_acceleration>1.0</max_wheel_acceleration>

  <!-- Output -->
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>

  <!-- Topics -->
  <command_topic>cmd_vel</command_topic>
  <odometry_topic>odom</odometry_topic>
</plugin>
\`\`\`

### Camera Plugin

\`\`\`xml
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>1920</width>
      <height>1080</height>
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
    <ros>
      <namespace>/robot</namespace>
      <remapping>image_raw:=camera/image</remapping>
      <remapping>camera_info:=camera/info</remapping>
    </ros>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
\`\`\`

### LiDAR Plugin

\`\`\`xml
<sensor name="lidar" type="ray">
  <pose>0 0 0.1 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>

  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
\`\`\`

## Controlling Robots from ROS 2

\`\`\`python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward velocity
        msg.angular.z = 0.2  # Turning rate
        self.publisher.publish(msg)
\`\`\`

## Practice Exercises

1. **Exercise 1**: Create a custom Gazebo world with obstacles
2. **Exercise 2**: Add a camera and LiDAR to your robot
3. **Exercise 3**: Write a node to control the robot using keyboard input

## Key Takeaways

✅ Gazebo provides realistic physics and sensor simulation
✅ Plugins enable ROS 2 integration
✅ Custom worlds allow testing in various environments
✅ Launch files simplify complex simulation setups
✅ Gazebo is essential for safe robot development
`
        },
        {
          id: "m2-c3",
          moduleId: "module-2",
          number: 3,
          title: "Unity Simulation and Robotics",
          content: `
# Unity Simulation and Robotics

## Why Unity for Robotics?

Unity offers advantages for robotics simulation:

- **Photorealistic Graphics**: Better than traditional robotics simulators
- **ML-Agents**: Built-in reinforcement learning framework
- **Cross-Platform**: Deploy to multiple platforms
- **Ecosystem**: Vast asset store and community

## Unity Robotics Hub

Unity Robotics Hub provides ROS integration:

### Installation

\`\`\`bash
# In Unity, use Package Manager to add:
# - ROS TCP Connector
# - URDF Importer
\`\`\`

### Setting Up ROS Connection

**In Unity (C#):**
\`\`\`csharp
using Unity.Robotics.ROSTCPConnector;

public class ROSConnection : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Connect();
    }
}
\`\`\`

**On ROS 2 side:**
\`\`\`bash
# Install ROS TCP endpoint
sudo apt install ros-humble-ros-tcp-endpoint

# Launch endpoint
ros2 run ros_tcp_endpoint default_server_endpoint
\`\`\`

## Importing URDF Models

Unity can directly import URDF files:

\`\`\`csharp
using Unity.Robotics.UrdfImporter;

public class RobotSpawner : MonoBehaviour
{
    void Start()
    {
        string urdfPath = "Assets/Robots/my_robot.urdf";
        UrdfRobotExtensions.Create(urdfPath);
    }
}
\`\`\`

## Publishing and Subscribing

### Publishing Camera Images to ROS

\`\`\`csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public Camera unityCamera;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>("camera/image");
    }

    void Update()
    {
        if (Time.frameCount % 3 == 0) // 10 Hz at 30 FPS
        {
            PublishImage();
        }
    }

    void PublishImage()
    {
        RenderTexture rt = RenderTexture.GetTemporary(
            unityCamera.pixelWidth,
            unityCamera.pixelHeight,
            24
        );

        unityCamera.targetTexture = rt;
        unityCamera.Render();

        Texture2D image = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        RenderTexture.active = rt;
        image.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        image.Apply();

        ImageMsg msg = new ImageMsg
        {
            height = (uint)rt.height,
            width = (uint)rt.width,
            encoding = "rgb8",
            data = image.GetRawTextureData()
        };

        ros.Publish("camera/image", msg);

        RenderTexture.ReleaseTemporary(rt);
    }
}
\`\`\`

### Subscribing to Velocity Commands

\`\`\`csharp
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;
    public float wheelRadius = 0.1f;
    public float wheelBase = 0.5f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("cmd_vel", ExecuteVelocityCommand);
    }

    void ExecuteVelocityCommand(TwistMsg msg)
    {
        float linearVel = (float)msg.linear.x;
        float angularVel = (float)msg.angular.z;

        // Differential drive kinematics
        float leftVel = (linearVel - angularVel * wheelBase / 2) / wheelRadius;
        float rightVel = (linearVel + angularVel * wheelBase / 2) / wheelRadius;

        // Convert to degrees per second
        leftVel *= Mathf.Rad2Deg;
        rightVel *= Mathf.Rad2Deg;

        SetWheelVelocity(leftWheel, leftVel);
        SetWheelVelocity(rightWheel, rightVel);
    }

    void SetWheelVelocity(ArticulationBody wheel, float velocity)
    {
        var drive = wheel.xDrive;
        drive.targetVelocity = velocity;
        wheel.xDrive = drive;
    }
}
\`\`\`

## Unity ML-Agents for Robot Learning

ML-Agents enables reinforcement learning in Unity:

### Setting Up an Agent

\`\`\`csharp
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class RobotAgent : Agent
{
    public Transform target;
    public Transform robot;

    public override void OnEpisodeBegin()
    {
        // Reset robot position
        robot.localPosition = new Vector3(0, 0.5f, 0);

        // Randomize target position
        target.localPosition = new Vector3(
            Random.Range(-4f, 4f),
            0.5f,
            Random.Range(-4f, 4f)
        );
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Robot position and velocity
        sensor.AddObservation(robot.localPosition);
        sensor.AddObservation(robot.GetComponent<Rigidbody>().velocity);

        // Target position
        sensor.AddObservation(target.localPosition);

        // Distance to target
        sensor.AddObservation(Vector3.Distance(robot.localPosition, target.localPosition));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Get actions
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];

        // Apply forces
        Rigidbody rb = robot.GetComponent<Rigidbody>();
        rb.AddForce(new Vector3(moveX, 0, moveZ) * 10f);

        // Reward for getting closer to target
        float distanceToTarget = Vector3.Distance(robot.localPosition, target.localPosition);

        if (distanceToTarget < 1.5f)
        {
            SetReward(1.0f);
            EndEpisode();
        }

        // Penalty for falling off platform
        if (robot.localPosition.y < 0)
        {
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }
}
\`\`\`

### Training Configuration

**robot_config.yaml:**
\`\`\`yaml
behaviors:
  RobotAgent:
    trainer_type: ppo
    hyperparameters:
      batch_size: 128
      buffer_size: 2048
      learning_rate: 3.0e-4
      beta: 5.0e-3
      epsilon: 0.2
      lambd: 0.95
      num_epoch: 3
    network_settings:
      normalize: false
      hidden_units: 128
      num_layers: 2
    reward_signals:
      extrinsic:
        gamma: 0.99
        strength: 1.0
    max_steps: 500000
    time_horizon: 64
    summary_freq: 10000
\`\`\`

## Synthetic Data Generation

Unity excels at generating training data:

\`\`\`csharp
public class DataGenerator : MonoBehaviour
{
    public Camera[] cameras;
    public GameObject[] objects;
    public int imagesPerObject = 1000;

    IEnumerator GenerateDataset()
    {
        for (int objIdx = 0; objIdx < objects.Length; objIdx++)
        {
            for (int i = 0; i < imagesPerObject; i++)
            {
                // Randomize lighting
                RenderSettings.ambientIntensity = Random.Range(0.5f, 1.5f);

                // Randomize object position
                objects[objIdx].transform.position = GetRandomPosition();
                objects[objIdx].transform.rotation = Random.rotation;

                // Capture from multiple angles
                foreach (Camera cam in cameras)
                {
                    yield return new WaitForEndOfFrame();
                    CaptureImage(cam, objIdx, i);
                }
            }
        }
    }

    void CaptureImage(Camera cam, int objectId, int imageId)
    {
        RenderTexture rt = new RenderTexture(1920, 1080, 24);
        cam.targetTexture = rt;
        cam.Render();

        Texture2D image = new Texture2D(rt.width, rt.height);
        RenderTexture.active = rt;
        image.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        image.Apply();

        byte[] bytes = image.EncodeToPNG();
        File.WriteAllBytes($"dataset/obj{objectId}_img{imageId}.png", bytes);

        cam.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);
    }
}
\`\`\`

## Practice Exercises

1. **Exercise 1**: Import a robot URDF and connect it to ROS 2
2. **Exercise 2**: Create an ML-Agent that learns to navigate to a target
3. **Exercise 3**: Generate a synthetic dataset of 10,000 labeled images

## Key Takeaways

✅ Unity provides photorealistic simulation for robotics
✅ ROS integration enables real robot deployment
✅ ML-Agents simplifies reinforcement learning
✅ Synthetic data generation accelerates AI development
✅ Unity complements traditional robotics simulators
`
        }
      ]
    },
    {
      id: "module-3",
      number: 3,
      title: "The AI-Robot Brain (NVIDIA Isaac™)",
      description: "Harness GPU-accelerated simulation and AI for advanced robotics",
      chapters: [
        {
          id: "m3-c1",
          moduleId: "module-3",
          number: 1,
          title: "Introduction to NVIDIA Isaac",
          content: `
# Introduction to NVIDIA Isaac

## What is NVIDIA Isaac?

NVIDIA Isaac is a platform for robot development that includes:

- **Isaac Sim**: Photorealistic simulation powered by Omniverse
- **Isaac SDK**: Libraries for perception and navigation
- **Isaac ROS**: GPU-accelerated ROS 2 packages
- **Isaac Cortex**: Behavior framework for AI robots

## Why Isaac Sim?

### GPU-Accelerated Physics
- Simulate thousands of robots in parallel
- Real-time complex physics calculations
- Faster than CPU-only simulators

### Photorealistic Rendering
- RTX ray tracing
- Accurate sensor simulation
- Synthetic data generation for AI

### ROS 2 Integration
- Native ROS 2 support
- Seamless sim-to-real transfer
- Standard robotics workflows

## Installing Isaac Sim

### System Requirements
- Ubuntu 20.04 or 22.04
- NVIDIA RTX GPU (2070 or better)
- 32GB RAM recommended
- 50GB free disk space

### Installation

\`\`\`bash
# Download from NVIDIA website
# https://developer.nvidia.com/isaac-sim

# Or use Omniverse Launcher
# Install through Omniverse -> Isaac Sim

# Verify installation
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh --help
\`\`\`

## Your First Isaac Sim Scene

### Creating a Basic Scene

\`\`\`python
from isaacsim import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import RigidPrim
import numpy as np

# Create world
world = World()

# Add ground plane
world.scene.add_default_ground_plane()

# Add a cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0, 0, 1.0]),
        size=np.array([0.5, 0.5, 0.5]),
        color=np.array([0, 0.5, 1.0]),
    )
)

# Reset world
world.reset()

# Run simulation
for i in range(1000):
    world.step(render=True)

# Cleanup
simulation_app.close()
\`\`\`

## Loading Robots

### Using USD Files

\`\`\`python
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot

# Get Isaac Sim assets
assets_root_path = get_assets_root_path()

# Load Franka robot
robot_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

robot = world.scene.add(
    Robot(
        prim_path="/World/Franka",
        name="franka",
        usd_path=robot_path,
        position=np.array([0, 0, 0]),
    )
)
\`\`\`

### Importing URDF

\`\`\`python
from omni.isaac.urdf import _urdf

# Import URDF
urdf_interface = _urdf.acquire_urdf_interface()

import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.self_collision = False
import_config.create_physics_scene = True

result, prim_path = urdf_interface.parse_urdf(
    urdf_path="/path/to/robot.urdf",
    import_config=import_config,
)
\`\`\`

## ROS 2 Bridge

### Enable ROS 2 Communication

\`\`\`python
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS2 extension
enable_extension("omni.isaac.ros2_bridge")

import rclpy
from std_msgs.msg import String

# Initialize ROS 2
rclpy.init()

# Now Isaac Sim can communicate with ROS 2 nodes
\`\`\`

### Publishing Camera Data

\`\`\`python
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.ros2_bridge import create_camera_publisher

# Add camera to scene
camera_path = "/World/Camera"

# Create ROS 2 camera publisher
create_camera_publisher(
    camera_prim_path=camera_path,
    topic_name="/camera/image_raw",
    frame_id="camera_frame",
    encoding="rgb8",
)
\`\`\`

### Subscribing to Commands

\`\`\`python
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self, robot):
        self.robot = robot
        self.node = rclpy.create_node('isaac_robot_controller')

        self.subscription = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

    def velocity_callback(self, msg):
        # Convert Twist to robot actions
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Apply to differential drive
        wheel_radius = 0.1
        wheel_base = 0.5

        left_vel = (linear_velocity - angular_velocity * wheel_base / 2) / wheel_radius
        right_vel = (linear_velocity + angular_velocity * wheel_base / 2) / wheel_radius

        # Set joint velocities
        action = ArticulationAction(joint_velocities=[left_vel, right_vel])
        self.robot.apply_action(action)
\`\`\`

## GPU Acceleration Benefits

### Performance Comparison

| Task | CPU (Gazebo) | GPU (Isaac Sim) | Speedup |
|------|-------------|-----------------|---------|
| Single robot | 1x real-time | 10x real-time | 10x |
| 10 robots | 0.1x real-time | 5x real-time | 50x |
| 100 robots | 0.01x real-time | 1x real-time | 100x |
| Ray casting | 100 Hz | 10,000 Hz | 100x |

## Practice Exercises

1. **Exercise 1**: Create an Isaac Sim scene with a robot and obstacles
2. **Exercise 2**: Set up ROS 2 bridge and control robot with teleop
3. **Exercise 3**: Add a camera and visualize images in RViz2

## Key Takeaways

✅ Isaac Sim provides GPU-accelerated robotics simulation
✅ Photorealistic rendering enables better AI training
✅ Native ROS 2 support ensures compatibility
✅ USD format enables asset sharing and collaboration
✅ Massive performance gains over traditional simulators
`
        },
        {
          id: "m3-c2",
          moduleId: "module-3",
          number: 2,
          title: "Isaac Navigation Stack",
          content: `
# Isaac Navigation Stack

## Introduction to Navigation

Autonomous navigation requires:

1. **Localization**: Where am I?
2. **Mapping**: What's around me?
3. **Planning**: How do I get there?
4. **Control**: Execute the plan

## Isaac ROS Nav2

Isaac provides GPU-accelerated Nav2 packages:

### Installation

\`\`\`bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-navigation

# Dependencies
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
\`\`\`

## SLAM with Isaac ROS

### Visual SLAM

\`\`\`python
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.range_sensor import _range_sensor

# Create world
world = World()

# Add robot with camera and IMU
robot = world.scene.add(
    WheeledRobot(
        prim_path="/World/Robot",
        name="robot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    )
)

# Add stereo cameras for Visual SLAM
left_camera = world.scene.add_camera(
    prim_path="/World/Robot/stereo_left",
    position=np.array([0.1, 0.05, 0.1]),
)

right_camera = world.scene.add_camera(
    prim_path="/World/Robot/stereo_right",
    position=np.array([0.1, -0.05, 0.1]),
)
\`\`\`

### Launch Visual SLAM

\`\`\`bash
# Terminal 1: Launch Isaac Sim with robot

# Terminal 2: Launch Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
\`\`\`

### Visualize in RViz

\`\`\`bash
ros2 run rviz2 rviz2 -d isaac_vslam.rviz
\`\`\`

## Path Planning

### Setting Up Nav2 with Isaac

**nav2_params.yaml:**
\`\`\`yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5
      max_vel_theta: 1.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false

recovery_server:
  ros__parameters:
    use_sim_time: True
    recovery_plugins: ["spin", "backup", "wait"]
\`\`\`

### Launch Navigation

\`\`\`python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=['nav2_params.yaml']
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=['nav2_params.yaml']
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=['nav2_params.yaml']
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'bt_navigator',
                    'controller_server',
                    'planner_server'
                ]
            }]
        ),
    ])
\`\`\`

## Isaac ROS DNN Stereo Depth

GPU-accelerated depth estimation:

### Setup

\`\`\`bash
# Install Isaac ROS DNN Stereo Disparity
sudo apt install ros-humble-isaac-ros-dnn-stereo-depth
\`\`\`

### Launch Stereo Depth

\`\`\`python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='stereo_depth_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_dnn_stereo_depth',
                plugin='nvidia::isaac_ros::dnn_stereo_depth::DnnStereoDepthNode',
                name='dnn_stereo_depth',
                parameters=[{
                    'engine_file_path': '/workspaces/isaac_ros-dev/models/dnn_stereo_depth.engine',
                    'input_layer_width': 960,
                    'input_layer_height': 576,
                }],
                remappings=[
                    ('left/image_rect', '/stereo_left/image'),
                    ('right/image_rect', '/stereo_right/image'),
                    ('disparity', '/disparity'),
                ]
            ),
        ],
    )

    return LaunchDescription([container])
\`\`\`

## Obstacle Avoidance

### Dynamic Window Approach (DWA)

\`\`\`python
class ObstacleAvoider:
    def __init__(self, robot):
        self.robot = robot
        self.node = rclpy.create_node('obstacle_avoider')

        # Subscribe to laser scan
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publish velocity commands
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, scan_msg):
        # Find minimum distance
        min_distance = min(scan_msg.ranges)
        min_idx = scan_msg.ranges.index(min_distance)

        # Calculate angle to obstacle
        angle = scan_msg.angle_min + min_idx * scan_msg.angle_increment

        cmd = Twist()

        if min_distance < 0.5:  # Too close
            # Turn away from obstacle
            cmd.angular.z = -np.sign(angle) * 0.5
            cmd.linear.x = 0.0
        else:
            # Move forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
\`\`\`

## Multi-Robot Navigation

Isaac Sim can simulate multiple robots efficiently:

\`\`\`python
class MultiRobotNav:
    def __init__(self, num_robots=5):
        self.world = World()
        self.robots = []

        for i in range(num_robots):
            robot = self.world.scene.add(
                WheeledRobot(
                    prim_path=f"/World/Robot_{i}",
                    name=f"robot_{i}",
                    position=np.array([i * 2.0, 0, 0]),
                )
            )
            self.robots.append(robot)

            # Create namespace for each robot
            self.setup_robot_nav(robot, f"robot_{i}")

    def setup_robot_nav(self, robot, namespace):
        # Launch Nav2 with namespace
        # Each robot gets independent navigation stack
        pass
\`\`\`

## Isaac ROS Nvblox

Real-time 3D reconstruction:

\`\`\`bash
# Install Nvblox
sudo apt install ros-humble-isaac-ros-nvblox

# Launch Nvblox
ros2 launch isaac_ros_nvblox nvblox.launch.py
\`\`\`

**nvblox_config.yaml:**
\`\`\`yaml
nvblox_node:
  ros__parameters:
    # Input
    depth_topic: /camera/depth/image
    color_topic: /camera/color/image

    # Output
    mesh_topic: /nvblox/mesh
    map_slice_topic: /nvblox/map_slice

    # Parameters
    voxel_size: 0.05
    max_integration_distance: 7.0
    max_tsdf_update_hz: 10.0
    max_color_update_hz: 5.0
    max_mesh_update_hz: 5.0
\`\`\`

## Practice Exercises

1. **Exercise 1**: Set up Visual SLAM in Isaac Sim and create a map
2. **Exercise 2**: Configure Nav2 to navigate robot to waypoints
3. **Exercise 3**: Implement multi-robot navigation with collision avoidance

## Key Takeaways

✅ Isaac ROS provides GPU-accelerated navigation
✅ Visual SLAM enables mapping without LiDAR
✅ Nav2 integration ensures standard ROS 2 compatibility
✅ Real-time depth estimation with DNN Stereo
✅ Nvblox enables 3D reconstruction for rich maps
`
        },
        {
          id: "m3-c3",
          moduleId: "module-3",
          number: 3,
          title: "Isaac Perception and Manipulation",
          content: `
# Isaac Perception and Manipulation

## Computer Vision for Robotics

Robots need to perceive their environment accurately. Isaac provides GPU-accelerated perception.

## Object Detection with Isaac ROS

### DOPE (Deep Object Pose Estimation)

DOPE estimates 6D poses of known objects:

\`\`\`bash
# Install Isaac ROS DOPE
sudo apt install ros-humble-isaac-ros-dope

# Download pre-trained models
cd ~/workspaces/isaac_ros-dev
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation
\`\`\`

### Configure DOPE

**dope_config.yaml:**
\`\`\`yaml
dope_encoder:
  ros__parameters:
    network_image_width: 640
    network_image_height: 480

dope_decoder:
  ros__parameters:
    object_name: "Ketchup"

    # 6D pose estimation parameters
    map_peak_threshold: 0.1
    peak_threshold: 0.1
\`\`\`

### Launch DOPE

\`\`\`python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    dope_encoder = ComposableNode(
        package='isaac_ros_dnn_encoders',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoder',
        name='dope_encoder',
        parameters=[{
            'network_image_width': 640,
            'network_image_height': 480,
        }],
        remappings=[
            ('encoded_tensor', 'tensor_pub'),
            ('image', '/camera/image_raw'),
        ]
    )

    dope_inference = ComposableNode(
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        name='dope_inference',
        parameters=[{
            'model_file_path': '/models/dope_ketchup.onnx',
            'engine_file_path': '/models/dope_ketchup.plan',
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['input'],
            'output_tensor_names': ['output'],
            'output_binding_names': ['output'],
        }]
    )

    dope_decoder = ComposableNode(
        package='isaac_ros_dope',
        plugin='nvidia::isaac_ros::dope::DopeDecoderNode',
        name='dope_decoder',
        parameters=[{
            'object_name': 'Ketchup',
        }]
    )

    container = ComposableNodeContainer(
        name='dope_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            dope_encoder,
            dope_inference,
            dope_decoder
        ],
    )

    return LaunchDescription([container])
\`\`\`

## Semantic Segmentation

### Isaac ROS U-Net

\`\`\`bash
# Install U-Net
sudo apt install ros-humble-isaac-ros-unet
\`\`\`

### Running Segmentation

\`\`\`python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class SegmentationNode:
    def __init__(self):
        self.node = rclpy.create_node('segmentation_processor')

        self.subscription = self.node.create_subscription(
            Image,
            '/unet/raw_segmentation_mask',
            self.segmentation_callback,
            10
        )

        self.bridge = CvBridge()

    def segmentation_callback(self, msg):
        # Convert to numpy array
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Class labels
        classes = {
            0: 'background',
            1: 'person',
            2: 'table',
            3: 'object',
        }

        # Find objects of interest
        object_pixels = np.where(mask == 3)

        if len(object_pixels[0]) > 0:
            # Calculate centroid
            centroid_y = np.mean(object_pixels[0])
            centroid_x = np.mean(object_pixels[1])

            self.node.get_logger().info(
                f'Object detected at ({centroid_x:.0f}, {centroid_y:.0f})'
            )
\`\`\`

## Manipulation with Isaac Sim

### Adding a Manipulator

\`\`\`python
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid

# Create world
world = World()

# Add Franka Panda robot
assets_root = get_assets_root_path()
franka = world.scene.add(
    Franka(
        prim_path="/World/Franka",
        name="franka",
    )
)

# Add object to manipulate
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0.5, 0, 0.05]),
        size=np.array([0.05, 0.05, 0.05]),
        color=np.array([1, 0, 0]),
    )
)

world.reset()
\`\`\`

### Motion Planning with MoveIt 2

\`\`\`python
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveGroupAction
import actionlib

class PickAndPlace:
    def __init__(self):
        self.node = rclpy.create_node('pick_and_place')

        # MoveIt action client
        self.move_group_client = actionlib.SimpleActionClient(
            'move_action',
            MoveGroupAction
        )

    def pick(self, object_pose):
        # 1. Move above object
        above_pose = object_pose.copy()
        above_pose.position.z += 0.1
        self.move_to_pose(above_pose)

        # 2. Move down to grasp
        self.move_to_pose(object_pose)

        # 3. Close gripper
        self.close_gripper()

        # 4. Lift object
        self.move_to_pose(above_pose)

    def place(self, target_pose):
        # 1. Move above target
        above_pose = target_pose.copy()
        above_pose.position.z += 0.1
        self.move_to_pose(above_pose)

        # 2. Move down
        self.move_to_pose(target_pose)

        # 3. Open gripper
        self.open_gripper()

        # 4. Retreat
        self.move_to_pose(above_pose)

    def move_to_pose(self, pose):
        goal = MoveGroupGoal()
        goal.request.group_name = "panda_arm"
        goal.request.pose_target = pose

        self.move_group_client.send_goal(goal)
        self.move_group_client.wait_for_result()
\`\`\`

## Grasp Planning

### Calculating Grasp Poses

\`\`\`python
import numpy as np
from scipy.spatial.transform import Rotation

class GraspPlanner:
    def __init__(self):
        self.gripper_width = 0.08

    def plan_parallel_jaw_grasp(self, object_pose, object_size):
        """
        Plan a grasp for a parallel jaw gripper
        """
        grasps = []

        # Grasp from top
        grasp = Pose()
        grasp.position.x = object_pose.position.x
        grasp.position.y = object_pose.position.y
        grasp.position.z = object_pose.position.z + object_size[2]/2 + 0.1

        # Orient gripper downward
        r = Rotation.from_euler('xyz', [np.pi, 0, 0])
        quat = r.as_quat()
        grasp.orientation.x = quat[0]
        grasp.orientation.y = quat[1]
        grasp.orientation.z = quat[2]
        grasp.orientation.w = quat[3]

        grasps.append(grasp)

        # Grasp from sides
        for angle in [0, np.pi/2, np.pi, 3*np.pi/2]:
            grasp = Pose()
            grasp.position.x = object_pose.position.x + 0.15 * np.cos(angle)
            grasp.position.y = object_pose.position.y + 0.15 * np.sin(angle)
            grasp.position.z = object_pose.position.z + object_size[2]/2

            r = Rotation.from_euler('xyz', [0, np.pi/2, angle])
            quat = r.as_quat()
            grasp.orientation.x = quat[0]
            grasp.orientation.y = quat[1]
            grasp.orientation.z = quat[2]
            grasp.orientation.w = quat[3]

            grasps.append(grasp)

        return grasps

    def filter_valid_grasps(self, grasps, object_size):
        """
        Filter grasps based on gripper constraints
        """
        valid_grasps = []

        for grasp in grasps:
            # Check if object fits in gripper
            if max(object_size[:2]) < self.gripper_width:
                valid_grasps.append(grasp)

        return valid_grasps
\`\`\`

## TensorRT Optimization

Convert models for faster inference:

\`\`\`python
import tensorrt as trt
import torch

class ModelOptimizer:
    def __init__(self):
        self.logger = trt.Logger(trt.Logger.WARNING)

    def convert_to_tensorrt(self, onnx_path, engine_path):
        """
        Convert ONNX model to TensorRT engine
        """
        builder = trt.Builder(self.logger)
        network = builder.create_network(
            1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        )

        parser = trt.OnnxParser(network, self.logger)

        # Parse ONNX
        with open(onnx_path, 'rb') as model:
            if not parser.parse(model.read()):
                for error in range(parser.num_errors):
                    print(parser.get_error(error))
                return None

        # Build engine
        config = builder.create_builder_config()
        config.max_workspace_size = 1 << 30  # 1GB

        # Enable FP16 if available
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)

        engine = builder.build_engine(network, config)

        # Save engine
        with open(engine_path, 'wb') as f:
            f.write(engine.serialize())

        return engine
\`\`\`

## Practice Exercises

1. **Exercise 1**: Set up DOPE to detect objects in Isaac Sim
2. **Exercise 2**: Implement pick-and-place with Franka robot
3. **Exercise 3**: Optimize a PyTorch model with TensorRT

## Key Takeaways

✅ Isaac ROS provides GPU-accelerated perception
✅ DOPE enables accurate 6D pose estimation
✅ MoveIt 2 integration simplifies motion planning
✅ Grasp planning generates feasible grasps
✅ TensorRT optimization accelerates inference by 5-10x
`
        }
      ]
    },
    {
      id: "module-4",
      number: 4,
      title: "Vision-Language-Action (VLA)",
      description: "Build robots that understand language and visual commands",
      chapters: [
        {
          id: "m4-c1",
          moduleId: "module-4",
          number: 1,
          title: "Vision-Language Models for Robotics",
          content: `
# Vision-Language Models for Robotics

## The VLA Revolution

Vision-Language-Action (VLA) models represent a paradigm shift in robotics:

- **Vision**: Understand visual scenes
- **Language**: Process natural language commands
- **Action**: Generate robot actions

## Foundation Models for Robotics

### CLIP (Contrastive Language-Image Pre-training)

CLIP learns joint vision-language representations:

\`\`\`python
import torch
import clip
from PIL import Image

# Load CLIP model
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

# Load image
image = preprocess(Image.open("robot_scene.jpg")).unsqueeze(0).to(device)

# Define possible actions
text_prompts = [
    "pick up the red block",
    "move to the table",
    "open the door",
    "grasp the cup"
]

text = clip.tokenize(text_prompts).to(device)

# Get similarities
with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text)

    # Normalize features
    image_features = image_features / image_features.norm(dim=-1, keepdim=True)
    text_features = text_features / text_features.norm(dim=-1, keepdim=True)

    # Calculate similarity
    similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)

print("Action probabilities:")
for i, prompt in enumerate(text_prompts):
    print(f"{prompt}: {similarity[0, i].item():.2%}")
\`\`\`

### Object Detection with CLIP

\`\`\`python
class CLIPObjectDetector:
    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)

    def find_object(self, image, object_descriptions):
        """
        Find which object description best matches the image
        """
        image_input = self.preprocess(image).unsqueeze(0).to(self.device)
        text_inputs = clip.tokenize(object_descriptions).to(self.device)

        with torch.no_grad():
            image_features = self.model.encode_image(image_input)
            text_features = self.model.encode_text(text_inputs)

            similarity = (image_features @ text_features.T).squeeze()

        best_match_idx = similarity.argmax().item()
        confidence = similarity[best_match_idx].item()

        return object_descriptions[best_match_idx], confidence

    def segment_and_classify(self, image, proposals, labels):
        """
        Classify image regions using CLIP
        """
        results = []

        for region in proposals:
            x1, y1, x2, y2 = region
            crop = image.crop((x1, y1, x2, y2))

            label, confidence = self.find_object(crop, labels)
            results.append({
                'bbox': region,
                'label': label,
                'confidence': confidence
            })

        return results
\`\`\`

## Large Language Models for Task Planning

### Using LLMs for High-Level Planning

\`\`\`python
from openai import OpenAI

class RobotTaskPlanner:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)

    def plan_task(self, instruction, environment_description):
        """
        Convert natural language instruction to robot plan
        """
        prompt = f"""
You are a robot task planner. Given a high-level instruction and environment description,
break down the task into primitive robot actions.

Environment: {environment_description}
Instruction: {instruction}

Available actions:
- navigate(location)
- pick(object)
- place(object, location)
- open(container)
- close(container)

Provide a numbered list of actions:
"""

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a helpful robot task planning assistant."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.3,
        )

        plan = response.choices[0].message.content
        return self.parse_plan(plan)

    def parse_plan(self, plan_text):
        """
        Parse LLM output into structured actions
        """
        actions = []
        lines = plan_text.strip().split('\n')

        for line in lines:
            # Extract action from format: "1. navigate(kitchen)"
            if '(' in line and ')' in line:
                action_str = line.split('.', 1)[1].strip()
                actions.append(action_str)

        return actions

# Example usage
planner = RobotTaskPlanner(api_key="your-key")

environment = """
Kitchen with:
- Counter with coffee maker, cups, and sugar
- Refrigerator
- Table with chairs
"""

instruction = "Make me a cup of coffee"

plan = planner.plan_task(instruction, environment)
print("Execution plan:")
for i, action in enumerate(plan, 1):
    print(f"{i}. {action}")
\`\`\`

## RT-1: Robotics Transformer

RT-1 is Google's vision-language-action model:

### Model Architecture

\`\`\`python
import torch
import torch.nn as nn

class RT1Model(nn.Module):
    """
    Simplified RT-1 architecture
    """
    def __init__(self,
                 image_size=224,
                 num_actions=7,
                 action_bins=256):
        super().__init__()

        # Image encoder (EfficientNet-based)
        self.image_encoder = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((7, 7)),
        )

        # Language encoder (simple embedding)
        self.text_encoder = nn.Embedding(10000, 512)

        # Transformer for action prediction
        self.transformer = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(
                d_model=512,
                nhead=8,
                dim_feedforward=2048,
                dropout=0.1,
            ),
            num_layers=6
        )

        # Action head
        self.action_head = nn.Linear(512, num_actions * action_bins)
        self.num_actions = num_actions
        self.action_bins = action_bins

    def forward(self, images, text_tokens):
        # Encode images
        batch_size = images.shape[0]
        img_features = self.image_encoder(images)
        img_features = img_features.flatten(2).permute(2, 0, 1)  # (seq, batch, dim)

        # Encode text
        text_features = self.text_encoder(text_tokens)
        text_features = text_features.permute(1, 0, 2)  # (seq, batch, dim)

        # Combine features
        combined = torch.cat([text_features, img_features], dim=0)

        # Transform
        transformed = self.transformer(combined)

        # Predict actions (use first token)
        action_logits = self.action_head(transformed[0])
        action_logits = action_logits.view(batch_size, self.num_actions, self.action_bins)

        return action_logits
\`\`\`

## OpenVLA

Open-source vision-language-action model:

\`\`\`python
from transformers import AutoModelForVision2Seq, AutoProcessor
import torch

class OpenVLAController:
    def __init__(self, model_name="openvla/openvla-7b"):
        self.processor = AutoProcessor.from_pretrained(model_name)
        self.model = AutoModelForVision2Seq.from_pretrained(
            model_name,
            torch_dtype=torch.bfloat16,
            device_map="auto",
        )

    def predict_action(self, image, instruction):
        """
        Predict robot action from image and language instruction
        """
        # Prepare inputs
        inputs = self.processor(
            images=image,
            text=instruction,
            return_tensors="pt"
        ).to(self.model.device)

        # Generate action
        with torch.no_grad():
            outputs = self.model.generate(
                **inputs,
                max_new_tokens=100,
                do_sample=False,
            )

        # Decode action
        action = self.processor.batch_decode(
            outputs,
            skip_special_tokens=True
        )[0]

        return self.parse_action(action)

    def parse_action(self, action_str):
        """
        Parse action string into robot commands
        """
        # Example: "move_to(0.5, 0.3, 0.2), grasp()"
        actions = []
        for cmd in action_str.split(','):
            cmd = cmd.strip()
            if 'move_to' in cmd:
                # Extract coordinates
                coords = cmd[cmd.find('(')+1:cmd.find(')')].split(',')
                actions.append({
                    'type': 'move',
                    'position': [float(x) for x in coords]
                })
            elif 'grasp' in cmd:
                actions.append({'type': 'grasp'})
            elif 'release' in cmd:
                actions.append({'type': 'release'})

        return actions
\`\`\`

## Practice Exercises

1. **Exercise 1**: Use CLIP to identify objects in robot camera feed
2. **Exercise 2**: Implement LLM-based task planner for kitchen tasks
3. **Exercise 3**: Train a simple RT-1 model on demonstration data

## Key Takeaways

✅ VLA models combine vision, language, and action
✅ CLIP enables zero-shot object recognition
✅ LLMs can decompose high-level tasks
✅ RT-1 demonstrates end-to-end learning
✅ OpenVLA provides open-source VLA capabilities
`
        },
        {
          id: "m4-c2",
          moduleId: "module-4",
          number: 2,
          title: "Language-Driven Robot Control",
          content: `
# Language-Driven Robot Control

## Natural Language for Robot Control

Natural language interfaces make robots accessible to non-experts.

## Grounding Language to Actions

### Semantic Parsing

\`\`\`python
import spacy
from dataclasses import dataclass

@dataclass
class RobotAction:
    verb: str
    object: str
    location: str = None

class LanguageGrounder:
    def __init__(self):
        self.nlp = spacy.load("en_core_web_sm")

        # Map verbs to robot primitives
        self.verb_mapping = {
            'pick': 'grasp',
            'grab': 'grasp',
            'get': 'grasp',
            'take': 'grasp',
            'move': 'navigate',
            'go': 'navigate',
            'put': 'place',
            'place': 'place',
            'open': 'open',
            'close': 'close',
        }

    def parse_command(self, command):
        """
        Parse natural language command into robot action
        """
        doc = self.nlp(command.lower())

        # Extract verb
        verb = None
        for token in doc:
            if token.pos_ == "VERB":
                verb = self.verb_mapping.get(token.lemma_, token.lemma_)
                break

        # Extract object
        obj = None
        for chunk in doc.noun_chunks:
            if chunk.root.dep_ in ["dobj", "pobj"]:
                obj = chunk.text
                break

        # Extract location
        location = None
        for token in doc:
            if token.dep_ == "prep":
                # Look for prepositional phrase
                for child in token.children:
                    if child.pos_ == "NOUN":
                        location = ' '.join([t.text for t in child.subtree])

        return RobotAction(verb=verb, object=obj, location=location)

# Example usage
grounder = LanguageGrounder()

commands = [
    "Pick up the red block",
    "Move to the kitchen",
    "Put the cup on the table",
    "Open the drawer",
]

for cmd in commands:
    action = grounder.parse_command(cmd)
    print(f"{cmd} -> {action}")
\`\`\`

### Advanced Grounding with LLMs

\`\`\`python
from openai import OpenAI
import json

class LLMGrounder:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)

    def ground_command(self, command, scene_description):
        """
        Ground natural language command using LLM
        """
        prompt = f"""
Given the following robot scene and natural language command,
extract the action, object, and any spatial information.

Scene: {scene_description}
Command: {command}

Respond in JSON format:
{{
    "action": "grasp|navigate|place|open|close",
    "object": "object name",
    "location": "location description or null",
    "parameters": {{additional parameters}}
}}
"""

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a robot command parser."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1,
        )

        result = json.loads(response.choices[0].message.content)
        return result

# Example
grounder = LLMGrounder(api_key="your-key")

scene = """
Objects visible:
- Red mug on the counter
- Blue book on the shelf
- Green plant by the window
- Table in the center
"""

command = "Bring me the red mug from the counter"
result = grounder.ground_command(command, scene)
print(json.dumps(result, indent=2))
\`\`\`

## Interactive Dialogue Systems

### Building a Robot Chat Interface

\`\`\`python
class RobotDialogueManager:
    def __init__(self, llm_api_key):
        self.client = OpenAI(api_key=llm_api_key)
        self.conversation_history = []
        self.robot_state = {
            'location': 'living room',
            'holding': None,
            'battery': 85,
        }

    def process_user_input(self, user_message):
        """
        Process user message and generate response
        """
        # Add user message to history
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })

        # Create context-aware prompt
        system_prompt = f"""
You are a helpful home robot assistant.

Current state:
- Location: {self.robot_state['location']}
- Holding: {self.robot_state['holding'] or 'nothing'}
- Battery: {self.robot_state['battery']}%

You can:
- Navigate to different rooms
- Pick up and move objects
- Answer questions about your state
- Ask for clarification when needed
"""

        messages = [
            {"role": "system", "content": system_prompt}
        ] + self.conversation_history

        # Get LLM response
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=messages,
            temperature=0.7,
        )

        assistant_message = response.choices[0].message.content

        # Add assistant response to history
        self.conversation_history.append({
            "role": "assistant",
            "content": assistant_message
        })

        # Check if robot needs to take action
        action = self.extract_action(assistant_message)
        if action:
            self.execute_action(action)

        return assistant_message

    def extract_action(self, message):
        """
        Extract executable action from assistant message
        """
        # Simple pattern matching (could use LLM for better extraction)
        if "navigate" in message.lower():
            # Extract destination
            words = message.split()
            try:
                idx = words.index("to")
                destination = ' '.join(words[idx+1:idx+3])
                return {"type": "navigate", "destination": destination}
            except:
                return None
        return None

    def execute_action(self, action):
        """
        Execute robot action and update state
        """
        if action['type'] == 'navigate':
            self.robot_state['location'] = action['destination']
            print(f"[Robot] Navigating to {action['destination']}...")

# Example conversation
dm = RobotDialogueManager(api_key="your-key")

conversation = [
    "Hi robot, where are you?",
    "Can you go to the kitchen?",
    "What's your battery level?",
    "Are you in the kitchen yet?",
]

for user_msg in conversation:
    print(f"User: {user_msg}")
    response = dm.process_user_input(user_msg)
    print(f"Robot: {response}\n")
\`\`\`

## Clarification and Error Handling

\`\`\`python
class SmartRobotAssistant:
    def __init__(self, llm_api_key):
        self.client = OpenAI(api_key=llm_api_key)

    def process_ambiguous_command(self, command, context):
        """
        Handle ambiguous commands by asking for clarification
        """
        prompt = f"""
Analyze this robot command for ambiguity:

Command: {command}
Context: {context}

If the command is ambiguous or missing critical information,
respond with a clarifying question. Otherwise, provide the parsed action.

Format:
{{
    "needs_clarification": true/false,
    "clarification_question": "question text or null",
    "parsed_action": {{action details}} or null
}}
"""

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a robot command analyzer."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.3,
        )

        result = json.loads(response.choices[0].message.content)
        return result

    def handle_failure(self, error_type, error_details):
        """
        Generate user-friendly error explanations
        """
        prompt = f"""
A robot encountered an error. Explain it to the user in simple terms
and suggest what they can do.

Error type: {error_type}
Details: {error_details}

Provide:
1. User-friendly explanation
2. Suggested next steps
"""

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a helpful robot error explainer."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.5,
        )

        return response.choices[0].message.content

# Example usage
assistant = SmartRobotAssistant(api_key="your-key")

# Test ambiguous command
result = assistant.process_ambiguous_command(
    "Pick up the cup",
    "There are 3 cups visible: red cup on table, blue cup on counter, white cup in sink"
)

if result['needs_clarification']:
    print(f"Robot: {result['clarification_question']}")
else:
    print(f"Action: {result['parsed_action']}")

# Test error handling
error_msg = assistant.handle_failure(
    "navigation_failed",
    "Path blocked by chair in doorway"
)
print(f"\nRobot: {error_msg}")
\`\`\`

## Multimodal Commands

Combining vision and language:

\`\`\`python
class MultimodalCommandProcessor:
    def __init__(self, clip_model, llm_api_key):
        self.clip_model = clip_model
        self.client = OpenAI(api_key=llm_api_key)

    def process_pointing_gesture(self, image, pointing_direction, verbal_command):
        """
        Process commands with pointing gestures
        """
        # Use CLIP to identify object in pointing direction
        # (simplified - would need actual pointing detection)

        candidates = self.detect_objects_in_direction(image, pointing_direction)

        # Use LLM to resolve reference
        prompt = f"""
User pointed in a direction and said: "{verbal_command}"
Detected objects in that direction: {candidates}

Which object are they referring to?
"""

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3,
        )

        target_object = response.choices[0].message.content
        return target_object
\`\`\`

## Practice Exercises

1. **Exercise 1**: Build a semantic parser for 20 common robot commands
2. **Exercise 2**: Implement a dialogue system with memory of past actions
3. **Exercise 3**: Create a clarification system for ambiguous references

## Key Takeaways

✅ Natural language interfaces make robots user-friendly
✅ Grounding maps language to robot primitives
✅ Dialogue systems enable interactive control
✅ Clarification handles ambiguity gracefully
✅ Multimodal processing combines vision and language
`
        },
        {
          id: "m4-c3",
          moduleId: "module-4",
          number: 3,
          title: "End-to-End VLA Systems",
          content: `
# End-to-End VLA Systems

## Building Complete VLA Systems

End-to-end VLA systems integrate vision, language, and action into a unified pipeline.

## System Architecture

\`\`\`python
import torch
import torch.nn as nn
from transformers import CLIPModel, GPT2LMHeadModel
from dataclasses import dataclass

@dataclass
class VLAOutput:
    actions: torch.Tensor
    confidence: float
    reasoning: str

class EndToEndVLA(nn.Module):
    """
    Complete Vision-Language-Action system
    """
    def __init__(self):
        super().__init__()

        # Vision encoder (CLIP)
        self.vision_encoder = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")

        # Language model (GPT-2)
        self.language_model = GPT2LMHeadModel.from_pretrained("gpt2")

        # Action decoder
        self.action_decoder = nn.Sequential(
            nn.Linear(768, 512),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, 7),  # 7-DOF robot arm
        )

        # Fusion layer
        self.fusion = nn.MultiheadAttention(
            embed_dim=512,
            num_heads=8,
            dropout=0.1
        )

    def forward(self, images, text_tokens, return_reasoning=False):
        """
        Forward pass through VLA system

        Args:
            images: (batch, 3, H, W)
            text_tokens: (batch, seq_len)
            return_reasoning: Whether to return intermediate reasoning
        """
        # Encode vision
        vision_outputs = self.vision_encoder.vision_model(images)
        vision_features = vision_outputs.last_hidden_state  # (batch, 49, 768)

        # Encode language
        language_outputs = self.language_model.transformer(text_tokens)
        language_features = language_outputs.last_hidden_state  # (batch, seq_len, 768)

        # Fuse modalities
        vision_proj = vision_features.permute(1, 0, 2)  # (49, batch, 768)
        language_proj = language_features.permute(1, 0, 2)  # (seq_len, batch, 768)

        fused, attention_weights = self.fusion(
            query=language_proj,
            key=vision_proj,
            value=vision_proj
        )

        # Decode actions
        fused_pooled = fused.mean(dim=0)  # (batch, 768)
        actions = self.action_decoder(fused_pooled)

        if return_reasoning:
            return actions, attention_weights
        return actions
\`\`\`

## Training VLA Models

### Data Collection

\`\`\`python
import h5py
import numpy as np

class RobotDemonstrationDataset:
    def __init__(self, h5_path):
        self.data = h5py.File(h5_path, 'r')
        self.episodes = list(self.data.keys())

    def __len__(self):
        return len(self.episodes)

    def __getitem__(self, idx):
        episode = self.data[self.episodes[idx]]

        # Load trajectory data
        observations = episode['observations'][:]  # (T, H, W, 3)
        actions = episode['actions'][:]  # (T, action_dim)
        language_instruction = episode.attrs['instruction']

        return {
            'observations': observations,
            'actions': actions,
            'instruction': language_instruction
        }

class VLADataCollector:
    """
    Collect demonstration data from teleoperation
    """
    def __init__(self, save_path):
        self.save_path = save_path
        self.episodes = []

    def start_episode(self, instruction):
        """
        Begin new demonstration episode
        """
        self.current_episode = {
            'instruction': instruction,
            'observations': [],
            'actions': [],
            'timestamps': []
        }

    def record_step(self, observation, action, timestamp):
        """
        Record single timestep
        """
        self.current_episode['observations'].append(observation)
        self.current_episode['actions'].append(action)
        self.current_episode['timestamps'].append(timestamp)

    def end_episode(self, success):
        """
        End episode and save if successful
        """
        if success:
            self.current_episode['success'] = True
            self.episodes.append(self.current_episode)

    def save_dataset(self):
        """
        Save all episodes to HDF5
        """
        with h5py.File(self.save_path, 'w') as f:
            for i, episode in enumerate(self.episodes):
                grp = f.create_group(f'episode_{i}')
                grp.create_dataset('observations', data=np.array(episode['observations']))
                grp.create_dataset('actions', data=np.array(episode['actions']))
                grp.create_dataset('timestamps', data=np.array(episode['timestamps']))
                grp.attrs['instruction'] = episode['instruction']
                grp.attrs['success'] = episode['success']
\`\`\`

### Training Loop

\`\`\`python
import torch.optim as optim
from torch.utils.data import DataLoader

class VLATrainer:
    def __init__(self, model, dataset, device='cuda'):
        self.model = model.to(device)
        self.device = device
        self.optimizer = optim.AdamW(model.parameters(), lr=1e-4)
        self.criterion = nn.MSELoss()

        self.dataloader = DataLoader(
            dataset,
            batch_size=16,
            shuffle=True,
            num_workers=4
        )

    def train_epoch(self):
        """
        Train for one epoch
        """
        self.model.train()
        total_loss = 0

        for batch in self.dataloader:
            images = batch['observations'].to(self.device)
            actions = batch['actions'].to(self.device)
            instructions = batch['instruction']

            # Tokenize instructions
            text_tokens = self.tokenize(instructions).to(self.device)

            # Forward pass
            predicted_actions = self.model(images, text_tokens)

            # Compute loss
            loss = self.criterion(predicted_actions, actions)

            # Backward pass
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            total_loss += loss.item()

        return total_loss / len(self.dataloader)

    def train(self, num_epochs):
        """
        Full training loop
        """
        for epoch in range(num_epochs):
            loss = self.train_epoch()
            print(f"Epoch {epoch+1}/{num_epochs}, Loss: {loss:.4f}")

            # Save checkpoint
            if (epoch + 1) % 10 == 0:
                self.save_checkpoint(f'checkpoint_epoch_{epoch+1}.pt')

    def save_checkpoint(self, path):
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
        }, path)
\`\`\`

## Deployment and Inference

### Real-Time Robot Control

\`\`\`python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

class VLARobotController(Node):
    def __init__(self, model_path):
        super().__init__('vla_controller')

        # Load trained model
        self.model = EndToEndVLA()
        checkpoint = torch.load(model_path)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.model.eval()
        self.model.to('cuda')

        # ROS setup
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.action_pub = self.create_publisher(
            Pose,
            '/robot/target_pose',
            10
        )

        self.current_instruction = "pick up the red block"
        self.latest_image = None

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def image_callback(self, msg):
        """
        Update latest camera image
        """
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def control_loop(self):
        """
        Run VLA model and publish actions
        """
        if self.latest_image is None:
            return

        # Preprocess image
        image_tensor = self.preprocess_image(self.latest_image)
        image_tensor = image_tensor.unsqueeze(0).to('cuda')

        # Tokenize instruction
        text_tokens = self.tokenize(self.current_instruction)
        text_tokens = text_tokens.unsqueeze(0).to('cuda')

        # Predict action
        with torch.no_grad():
            action = self.model(image_tensor, text_tokens)

        # Convert to robot command
        pose = self.action_to_pose(action[0])

        # Publish
        self.action_pub.publish(pose)

    def action_to_pose(self, action_tensor):
        """
        Convert model output to robot pose
        """
        pose = Pose()
        pose.position.x = action_tensor[0].item()
        pose.position.y = action_tensor[1].item()
        pose.position.z = action_tensor[2].item()
        pose.orientation.x = action_tensor[3].item()
        pose.orientation.y = action_tensor[4].item()
        pose.orientation.z = action_tensor[5].item()
        pose.orientation.w = action_tensor[6].item()
        return pose

def main():
    rclpy.init()
    controller = VLARobotController('vla_model.pt')
    rclpy.spin(controller)
    rclpy.shutdown()
\`\`\`

## Evaluation and Benchmarking

\`\`\`python
class VLAEvaluator:
    def __init__(self, model, test_tasks):
        self.model = model
        self.test_tasks = test_tasks

    def evaluate_task(self, task):
        """
        Evaluate model on single task
        """
        success = False
        steps = 0
        max_steps = 100

        # Reset environment
        obs = task.reset()

        while steps < max_steps:
            # Get action from model
            action = self.model.predict(obs, task.instruction)

            # Execute in environment
            obs, reward, done, info = task.step(action)
            steps += 1

            if done:
                success = info.get('success', False)
                break

        return {
            'success': success,
            'steps': steps,
            'task': task.name
        }

    def evaluate_all(self):
        """
        Evaluate on all test tasks
        """
        results = []

        for task in self.test_tasks:
            result = self.evaluate_task(task)
            results.append(result)
            print(f"{task.name}: {'✓' if result['success'] else '✗'} ({result['steps']} steps)")

        # Compute metrics
        success_rate = sum(r['success'] for r in results) / len(results)
        avg_steps = np.mean([r['steps'] for r in results if r['success']])

        print(f"\nOverall Success Rate: {success_rate:.2%}")
        print(f"Average Steps (successful): {avg_steps:.1f}")

        return results
\`\`\`

## Case Studies

### Home Service Robot

\`\`\`python
class HomeServiceVLA:
    """
    VLA system for home service tasks
    """
    def __init__(self):
        self.vla_model = EndToEndVLA()
        self.task_planner = RobotTaskPlanner()

    def execute_home_task(self, instruction):
        """
        Execute complex home tasks using VLA
        """
        # 1. High-level planning with LLM
        plan = self.task_planner.plan_task(
            instruction,
            self.get_environment_description()
        )

        # 2. Execute each step with VLA
        for step in plan:
            print(f"Executing: {step}")

            obs = self.get_observation()
            action = self.vla_model.predict(obs, step)

            self.execute_action(action)

            if not self.verify_completion(step):
                self.handle_failure(step)

# Example tasks
tasks = [
    "Set the table for dinner",
    "Clean up the living room",
    "Water the plants",
    "Bring me a snack from the kitchen"
]
\`\`\`

## Practice Exercises

1. **Exercise 1**: Collect 100 demonstrations and train a VLA model
2. **Exercise 2**: Deploy VLA model on real robot or simulation
3. **Exercise 3**: Evaluate on 10 different tasks and analyze failures

## Key Takeaways

✅ End-to-end VLA systems unify vision, language, and action
✅ Demonstration data is critical for training
✅ Real-time inference requires optimization
✅ Evaluation on diverse tasks measures generalization
✅ VLA represents the future of general-purpose robotics

## Congratulations!

You've completed the **Physical AI & Humanoid Robotics** course! You now have knowledge of:

- ROS 2 fundamentals and architecture
- Simulation with Gazebo, Unity, and Isaac
- GPU-accelerated robotics with NVIDIA Isaac
- Vision-language-action models for intelligent robots

Continue building, experimenting, and pushing the boundaries of robotics! 🤖
`
        }
      ]
    }
  ]
};

// Helper functions
export const getModuleById = (moduleId) => {
  return bookData.modules.find(m => m.id === moduleId);
};

export const getChapterById = (chapterId) => {
  for (const module of bookData.modules) {
    const chapter = module.chapters.find(c => c.id === chapterId);
    if (chapter) return chapter;
  }
  return null;
};

export const getAllChapters = () => {
  return bookData.modules.flatMap(m => m.chapters);
};

export const getNextChapter = (currentChapterId) => {
  const chapters = getAllChapters();
  const currentIndex = chapters.findIndex(c => c.id === currentChapterId);
  return currentIndex < chapters.length - 1 ? chapters[currentIndex + 1] : null;
};

export const getPreviousChapter = (currentChapterId) => {
  const chapters = getAllChapters();
  const currentIndex = chapters.findIndex(c => c.id === currentChapterId);
  return currentIndex > 0 ? chapters[currentIndex - 1] : null;
};

export const searchChapters = (query) => {
  const lowerQuery = query.toLowerCase();
  return getAllChapters().filter(chapter =>
    chapter.title.toLowerCase().includes(lowerQuery) ||
    chapter.content.toLowerCase().includes(lowerQuery)
  );
};
