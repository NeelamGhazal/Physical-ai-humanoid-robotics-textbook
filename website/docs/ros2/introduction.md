---
sidebar_position: 1
---

# Introduction to ROS 2: Foundation of Modern Robotics

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the core concepts of ROS 2 and its architecture
- Create and implement basic ROS 2 nodes using Python
- Explain the publisher-subscriber communication pattern
- Work with ROS 2 services and actions
- Understand URDF (Unified Robot Description Format) basics

## Real-World Context

ROS 2 (Robot Operating System 2) serves as the backbone for modern robotics development, providing standardized tools, libraries, and conventions that simplify the creation of complex robotic applications. From autonomous vehicles to humanoid robots, ROS 2 powers countless robotic systems worldwide. Understanding ROS 2 is essential for anyone pursuing a career in robotics, as it's used extensively in both academia and industry.

Major companies like Boston Dynamics, NASA, and numerous startups rely on ROS 2 for their robotic systems. The modular architecture allows for rapid prototyping and testing of robotic applications, making it invaluable for research and development in embodied AI.

## Introduction to ROS 2 Architecture

ROS 2 represents a significant evolution from its predecessor, addressing critical limitations in ROS 1 while maintaining the core philosophy of distributed computing for robotics. Unlike ROS 1 which relied on a centralized master, ROS 2 adopts a decentralized architecture based on DDS (Data Distribution Service) middleware, enabling improved reliability, security, and scalability.

The ROS 2 ecosystem consists of several key components that work together to provide a comprehensive robotics development platform. The fundamental building blocks include nodes, topics, services, actions, and parameters, each serving a specific role in the communication and coordination of robotic systems.

**Figure: ROS 2 node graph showing publisher-subscriber flow** - In this architectural diagram, we visualize how multiple nodes communicate through topics. A sensor node publishes data to a `/sensor_data` topic, which is subscribed to by both a processing node and a logging node. Meanwhile, a controller node publishes commands to an `/actuator_commands` topic, which is consumed by an actuator node. Additionally, a parameter server provides configuration parameters to all nodes in the system.

The decentralized nature of ROS 2 means that nodes can be distributed across multiple machines, connected through a network. This enables complex robotic systems where different computational tasks can be distributed across specialized hardware, from high-performance computers for perception algorithms to microcontrollers for real-time actuator control.

## Core Concepts: Nodes and Packages

A ROS 2 node is essentially a process that performs computation. Nodes are organized into packages, which provide a way to organize and distribute ROS 2 software. Each package contains executables (nodes), libraries, and other resources needed for specific functionality.

Let's examine a basic ROS 2 node implementation:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()

    # Use MultiThreadedExecutor to run both nodes simultaneously
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(minimal_publisher)
    executor.add_node(minimal_subscriber)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        minimal_publisher.destroy_node()
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This example demonstrates the fundamental publisher-subscriber pattern in ROS 2. The publisher node creates a timer that triggers every 0.5 seconds, sending a "Hello World" message with an incrementing counter. The subscriber node listens to the same topic and logs the received messages. The decentralized architecture means these nodes can run on different machines and still communicate seamlessly.

## Publisher-Subscriber Communication Pattern

The publisher-subscriber pattern is the primary communication mechanism in ROS 2. Publishers send messages to topics, and subscribers receive messages from topics. This loose coupling allows for flexible system architectures where publishers and subscribers don't need to know about each other's existence.

Topics in ROS 2 are named buses over which nodes exchange messages. Messages are datagrams that consist of typed fields (e.g., integers, floats, arrays, timestamps). The type system ensures that publishers and subscribers agree on the message format, preventing communication errors.

Quality of Service (QoS) profiles provide fine-grained control over communication characteristics. For example, you can specify reliability (reliable vs. best-effort), durability (volatile vs. transient-local), and history (keep-all vs. keep-last). These profiles are crucial for real-time systems where timing and reliability requirements vary depending on the data being transmitted.

## Services and Actions

While the publisher-subscriber pattern is ideal for continuous data streams, ROS 2 also provides services for request-response communication and actions for goal-oriented tasks with feedback.

Services provide synchronous request-response communication:

```python
# Service definition (in srv/AddTwoInts.srv)
int64 a
int64 b
---
int64 sum

# Service server implementation
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

Actions are used for long-running tasks that provide feedback and can be canceled:

```python
# Action definition (in action/Fibonacci.action)
int32 order
---
int32[] sequence
---
int32[] feedback

# Action server implementation
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci


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

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info('Feedback: {feedback_msg.sequence}')

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Understanding rclpy

rclpy is the Python client library for ROS 2, providing the interface between Python applications and the ROS 2 middleware. It handles the underlying DDS communication, allowing developers to focus on application logic.

The rclpy library provides several key functionalities:
- Node creation and management
- Publisher and subscriber creation
- Service and action client/server creation
- Parameter handling
- Timer creation for periodic callbacks
- Logging capabilities
- Lifecycle management

The library abstracts away the complexity of DDS while providing access to important features like Quality of Service settings, which are crucial for real-time robotic applications.

## URDF: Unified Robot Description Format

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the physical and visual properties of a robot, including links (rigid parts), joints (connections between links), inertial properties, visual meshes, and collision geometries.

**Figure: URDF robot model diagram showing links and joints** - This diagram illustrates a simple robot arm with three links connected by rotational joints. The base link is fixed, followed by a shoulder joint connecting to the upper arm link, an elbow joint connecting to the forearm link, and a wrist joint connecting to the end effector. Each link has associated visual and collision properties, with coordinate frames attached to each joint.

A basic URDF example:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Shoulder joint and link -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

URDF files are crucial for simulation, visualization, and kinematic calculations. They allow ROS 2 tools to understand the robot's structure, enabling features like inverse kinematics, collision detection, and robot state publishing.

## Working with Parameters

Parameters in ROS 2 provide a way to configure nodes at runtime. They can be set at launch time, modified during execution, and shared between nodes. Parameters are strongly typed and support various data types including integers, floats, strings, booleans, and lists.

Parameter management is crucial for configuring robotic systems without recompilation. For example, you might have parameters controlling PID controller gains, sensor calibration values, or operational modes that need to be adjusted based on environmental conditions.

## Conclusion

ROS 2 provides a comprehensive framework for developing complex robotic systems with standardized communication patterns, robust tooling, and flexible architecture. Understanding the core concepts of nodes, topics, services, actions, and URDF is essential for building reliable robotic applications. The decentralized architecture and Quality of Service features make ROS 2 suitable for both research and production environments.

In the next chapter, we'll explore how to integrate ROS 2 with simulation environments like Gazebo to test and validate our robotic applications in virtual worlds before deploying them to physical robots.

## Exercises

1. Create a ROS 2 package with a publisher that sends temperature readings and a subscriber that processes and logs these values.
2. Implement a service that calculates the Euclidean distance between two points in 3D space.
3. Design a simple URDF model for a wheeled robot with differential drive kinematics.
4. Experiment with different Quality of Service profiles for a real-time sensor data publisher.