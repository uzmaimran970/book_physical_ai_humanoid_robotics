---
id: module1-chapter2
title: ROS 2 Services and Actions
sidebar_label: Chapter 2 - Services & Actions
---

# ROS 2 Services and Actions

## Introduction

While topics are excellent for continuous data streams, robots often need different communication patterns. **Services** provide synchronous request-response interactions, while **Actions** handle long-running tasks with feedback and cancellation.

## ROS 2 Services

Services implement a client-server model where one node requests a service and waits for a response.

### When to Use Services

Use services when you need:
- **Synchronous communication**: Request and wait for response
- **One-to-one interaction**: Single client to single server
- **Computational tasks**: Image processing, path planning, state queries
- **Configuration changes**: Update parameters, reset systems

### Service vs. Topic

| Feature | Topic | Service |
|---------|-------|---------|
| Pattern | Publish-Subscribe | Request-Response |
| Communication | Asynchronous | Synchronous |
| Connections | Many-to-Many | One-to-One |
| Use Case | Continuous data | On-demand computation |

### Service Definition

Services are defined in `.srv` files with request and response sections:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Creating a Service Server

Here's a simple calculator service:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class CalculatorService(Node):
    def __init__(self):
        super().__init__('calculator_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback)
        self.get_logger().info('Calculator service ready')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Received: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service = CalculatorService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()
```

### Creating a Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class CalculatorClient(Node):
    def __init__(self):
        super().__init__('calculator_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for calculator service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service asynchronously
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = CalculatorClient()

    # Send request
    future = client.send_request(10, 20)

    # Wait for response
    rclpy.spin_until_future_complete(client, future)

    if future.result() is not None:
        print(f'Result: {future.result().sum}')
    else:
        print('Service call failed')

    client.destroy_node()
    rclpy.shutdown()
```

## Real-World Service Examples

### 1. Robot State Service

Query the current state of the robot:

```
# GetRobotState.srv
---
string state
float32 battery_level
geometry_msgs/Pose current_pose
bool is_emergency_stopped
```

### 2. Path Planning Service

Request a path from current position to goal:

```
# PlanPath.srv
geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped goal
---
nav_msgs/Path path
bool success
string message
```

### 3. Image Processing Service

Process an image and return results:

```
# DetectObjects.srv
sensor_msgs/Image image
---
DetectedObject[] objects
int32 num_detections
```

## ROS 2 Actions

Actions are designed for long-running tasks that need:
- **Feedback**: Progress updates during execution
- **Cancellation**: Ability to cancel ongoing tasks
- **Asynchronous execution**: Don't block while waiting

### Action Structure

Actions have three components:

1. **Goal**: What to accomplish
2. **Feedback**: Progress updates
3. **Result**: Final outcome

### Action Definition

Actions are defined in `.action` files:

```
# Navigate.action

# Goal
geometry_msgs/PoseStamped target_pose
---
# Result
bool success
float32 total_distance
duration total_time
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 estimated_time_remaining
```

### Creating an Action Server

Here's a Fibonacci action server that provides feedback:

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Generate Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if cancellation requested
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Calculate next number
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    server = FibonacciActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
```

### Creating an Action Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Wait for action server
        self._action_client.wait_for_server()

        # Send goal with callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = FibonacciActionClient()
    client.send_goal(10)
    rclpy.spin(client)
```

## Practical Robotics Actions

### 1. Navigation Action

Navigate robot to a target pose with continuous feedback:

```python
# Goal: Target pose
# Feedback: Current pose, distance remaining, ETA
# Result: Success status, final pose, path taken

class NavigationAction(Node):
    def execute_callback(self, goal_handle):
        target = goal_handle.request.target_pose

        while not self.reached_goal(target):
            # Publish feedback
            feedback = NavigateToGoal.Feedback()
            feedback.current_pose = self.get_current_pose()
            feedback.distance_remaining = self.calculate_distance(target)
            goal_handle.publish_feedback(feedback)

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return NavigateToGoal.Result()

            time.sleep(0.1)

        # Success
        goal_handle.succeed()
        result = NavigateToGoal.Result()
        result.success = True
        return result
```

### 2. Gripper Control Action

Control robot gripper with force feedback:

```
# GripperControl.action

# Goal
float32 target_position  # 0.0 (closed) to 1.0 (open)
float32 max_force
---
# Result
float32 final_position
float32 applied_force
bool object_detected
---
# Feedback
float32 current_position
float32 current_force
```

### 3. Arm Trajectory Action

Execute arm trajectory with waypoint feedback:

```
# ArmTrajectory.action

# Goal
trajectory_msgs/JointTrajectory trajectory
---
# Result
bool success
string error_message
---
# Feedback
int32 current_waypoint
float32 progress_percentage
```

## Command-Line Tools

### Service Commands

```bash
# List available services
ros2 service list

# Show service type
ros2 service type /add_two_ints

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"

# Get service details
ros2 service info /add_two_ints
```

### Action Commands

```bash
# List available actions
ros2 action list

# Show action info
ros2 action info /fibonacci

# Send action goal
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# Send goal with feedback
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 10}" --feedback
```

## Decision Matrix: Topics vs Services vs Actions

### Use **Topics** when:
- ✅ Continuous data stream (sensor readings)
- ✅ Multiple subscribers needed
- ✅ Low latency required
- ✅ Fire-and-forget pattern

### Use **Services** when:
- ✅ Request-response pattern needed
- ✅ Task completes quickly (< 1 second)
- ✅ No progress feedback required
- ✅ Computation or query

### Use **Actions** when:
- ✅ Long-running task (> 1 second)
- ✅ Progress feedback needed
- ✅ Cancellation required
- ✅ Preemptable operations

## Best Practices

### 1. Service Timeout Handling

```python
def call_service_with_timeout(self, request, timeout=5.0):
    future = self.client.call_async(request)
    rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

    if future.result() is not None:
        return future.result()
    else:
        self.get_logger().error('Service call timed out')
        return None
```

### 2. Action Cancellation

```python
def cancel_goal(self):
    if self._goal_handle is not None:
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)
```

### 3. Error Handling

```python
def execute_callback(self, goal_handle):
    try:
        # Execute action
        result = self.perform_task(goal_handle)
        goal_handle.succeed()
        return result
    except Exception as e:
        self.get_logger().error(f'Action failed: {str(e)}')
        goal_handle.abort()
        return ResultType()
```

## Key Takeaways

- **Services** provide synchronous request-response communication
- **Actions** handle long-running tasks with feedback and cancellation
- Choose the right communication pattern based on task requirements
- Use timeouts and error handling for robust systems
- Command-line tools help test services and actions during development

## Next Steps

In the next chapter, you'll learn about:
- ROS 2 launch files for starting multiple nodes
- Parameter management and configuration
- Creating reusable robot systems
