---
id: module1-chapter1
title: ROS 2 Nodes and Topics
sidebar_label: Chapter 1 - Nodes & Topics
---

# ROS 2 Nodes and Topics

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) is the next-generation robotics middleware framework that serves as the nervous system for modern robots. Unlike its predecessor ROS 1, ROS 2 is built on the Data Distribution Service (DDS) standard, providing real-time performance, improved security, and multi-robot communication capabilities.

### Why ROS 2?

- **Real-time Performance**: Built for time-critical robotic applications
- **Cross-platform Support**: Works on Linux, Windows, and macOS
- **Security**: Built-in security features using DDS-Security
- **Scalability**: Supports single robots to multi-robot fleets
- **Industry Adoption**: Used in autonomous vehicles, drones, and industrial robots

## Understanding ROS 2 Nodes

A **node** is a fundamental building block in ROS 2. Each node is an independent process that performs a specific computation or task.

### What is a Node?

Think of nodes as specialized workers in a factory:
- Each worker (node) has a specific job
- Workers communicate with each other to complete complex tasks
- Workers can be added or removed without shutting down the entire factory

### Node Characteristics

1. **Independence**: Each node runs as a separate process
2. **Modularity**: Nodes can be reused across different robots
3. **Fault Isolation**: If one node crashes, others continue running
4. **Language Flexibility**: Can be written in C++, Python, or other supported languages

### Creating Your First Node

Here's a simple Python node that publishes "Hello, Robot!" messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher = self.create_publisher(String, 'hello_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello, Robot!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Node Lifecycle

ROS 2 introduces a managed node lifecycle with distinct states:

1. **Unconfigured**: Node is loaded but not ready
2. **Inactive**: Node is configured but not processing data
3. **Active**: Node is fully operational
4. **Finalized**: Node is shutting down

## Understanding Topics

**Topics** are named buses over which nodes exchange messages. They implement a publish-subscribe pattern.

### The Publish-Subscribe Pattern

```
Publisher Node → Topic → Subscriber Node(s)
```

- **Publishers** send data to topics
- **Subscribers** receive data from topics
- **Many-to-Many**: Multiple publishers and subscribers can use the same topic

### Topic Naming Conventions

Follow these best practices for topic names:

- Use lowercase with underscores: `camera_image`
- Use namespaces for organization: `/robot1/sensors/camera`
- Be descriptive: `/arm/joint_states` instead of `/data`
- Avoid special characters except `/` and `_`

### Common ROS 2 Topics

| Topic Name | Message Type | Purpose |
|------------|--------------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/scan` | `sensor_msgs/LaserScan` | Laser scanner data |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera images |
| `/joint_states` | `sensor_msgs/JointState` | Robot joint positions |

## Message Types

Messages are the data structures sent over topics. ROS 2 provides standard message types and allows custom definitions.

### Standard Message Packages

1. **std_msgs**: Basic data types (String, Int32, Float64)
2. **geometry_msgs**: Poses, velocities, transforms
3. **sensor_msgs**: Sensor data (Image, LaserScan, IMU)
4. **nav_msgs**: Navigation data (Odometry, Path)

### Creating Custom Messages

Define custom messages in `.msg` files:

```
# RobotStatus.msg
string robot_name
float32 battery_level
bool is_moving
geometry_msgs/Pose current_pose
```

### Message Quality of Service (QoS)

ROS 2 introduces QoS policies for reliable communication:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Transient local vs. volatile
- **History**: Keep last N messages
- **Deadline**: Maximum time between messages

## Practical Exercises

### Exercise 1: Create a Temperature Publisher

Create a node that publishes simulated temperature readings:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(0.5, self.publish_temperature)

    def publish_temperature(self):
        msg = Float32()
        msg.data = 20.0 + random.uniform(-5.0, 5.0)
        self.publisher.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data:.2f}°C')
```

### Exercise 2: Create a Temperature Subscriber

Create a node that monitors temperature and sends alerts:

```python
class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10)

    def temperature_callback(self, msg):
        if msg.data > 25.0:
            self.get_logger().warn(f'High temperature: {msg.data:.2f}°C')
        else:
            self.get_logger().info(f'Normal temperature: {msg.data:.2f}°C')
```

### Exercise 3: List Active Topics

Use command-line tools to inspect your ROS 2 system:

```bash
# List all active topics
ros2 topic list

# Show topic details
ros2 topic info /temperature

# Echo messages from a topic
ros2 topic echo /temperature

# Show message frequency
ros2 topic hz /temperature

# Publish from command line
ros2 topic pub /temperature std_msgs/Float32 "data: 23.5"
```

## Common Patterns and Best Practices

### 1. Namespacing for Multi-Robot Systems

```python
class RobotNode(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_controller')
        topic_name = f'/{robot_name}/cmd_vel'
        self.publisher = self.create_publisher(Twist, topic_name, 10)
```

### 2. Parameter-Based Configuration

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        self.declare_parameter('publish_rate', 10.0)
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0/rate, self.callback)
```

### 3. Graceful Shutdown

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Debugging and Monitoring

### Essential Tools

1. **rqt_graph**: Visualize node and topic connections
   ```bash
   ros2 run rqt_graph rqt_graph
   ```

2. **ros2 node**: Inspect node information
   ```bash
   ros2 node list
   ros2 node info /my_node
   ```

3. **ros2 topic**: Monitor topic communication
   ```bash
   ros2 topic list
   ros2 topic echo /my_topic
   ros2 topic hz /my_topic
   ```

4. **ros2 bag**: Record and replay data
   ```bash
   ros2 bag record -a  # Record all topics
   ros2 bag play my_recording.db3
   ```

## Key Takeaways

- **Nodes** are independent processes that perform specific tasks
- **Topics** enable publish-subscribe communication between nodes
- **Messages** are typed data structures exchanged over topics
- **QoS policies** provide fine-grained control over message delivery
- **Command-line tools** are essential for development and debugging
- **Namespacing** enables multi-robot systems
- **Best practices** improve code maintainability and scalability

## Next Steps

In the next chapter, you'll learn about:
- ROS 2 Services for request-response communication
- Actions for long-running tasks with feedback
- When to use topics vs. services vs. actions
