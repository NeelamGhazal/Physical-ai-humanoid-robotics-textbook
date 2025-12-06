---
sidebar_position: 2
---

# ROS 2 Topics and rclpy: Deep Dive into Communication

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement advanced publisher-subscriber patterns in ROS 2
- Utilize rclpy for complex node communication scenarios
- Configure Quality of Service settings for different communication needs
- Create custom message types for specialized applications
- Implement latching and transient local communication patterns

## Real-World Context

Understanding advanced ROS 2 communication patterns is crucial for developing robust robotic systems. In real-world applications, different types of data require different communication characteristics. Sensor data might need real-time delivery with best-effort reliability, while critical control commands might require guaranteed delivery. The ability to configure these patterns appropriately can mean the difference between a successful robotic deployment and one that fails under operational conditions.

Industrial robots, autonomous vehicles, and humanoid robots all rely on carefully designed communication patterns to coordinate complex behaviors. For example, a humanoid robot walking through a room needs to balance real-time sensor processing with reliable command execution to actuators, requiring different QoS settings for different data streams.

## Advanced Publisher-Subscriber Patterns

ROS 2 provides several advanced patterns beyond the basic publisher-subscriber model to handle complex communication requirements. These include latched topics for providing initial state to new subscribers, transient local durability for persistent state, and various reliability and deadline settings.

**Figure: Advanced ROS 2 communication patterns showing latched topics and QoS settings** - This diagram illustrates different communication patterns: a latched topic that maintains the last published message for new subscribers, a best-effort topic for sensor data where occasional packet loss is acceptable, and a reliable topic for critical commands where all messages must be delivered.

Latched topics are particularly useful for state information that new subscribers need to know immediately upon connecting. For example, a robot's current pose or configuration parameters might be published to a latched topic so that any new node that subscribes immediately receives the current state rather than waiting for the next update.

Here's an example of implementing a latched publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class LatchedPublisher(Node):

    def __init__(self):
        super().__init__('latched_publisher')

        # Create a QoS profile with transient local durability for latching
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(String, 'latched_topic', qos_profile)

        # Publish initial state
        msg = String()
        msg.data = 'Initial robot state: IDLE'
        self.publisher_.publish(msg)

        # Timer for periodic updates
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.state_counter = 0

    def timer_callback(self):
        msg = String()
        states = ['IDLE', 'ACTIVE', 'BUSY', 'ERROR']
        msg.data = f'Robot state: {states[self.state_counter % len(states)]}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.state_counter += 1


class LatchedSubscriber(Node):

    def __init__(self):
        super().__init__('latched_subscriber')

        # Same QoS profile as publisher
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.subscription = self.create_subscription(
            String,
            'latched_topic',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received latched state: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    latched_publisher = LatchedPublisher()
    latched_subscriber = LatchedSubscriber()

    # Use MultiThreadedExecutor to run both nodes simultaneously
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(latched_publisher)
    executor.add_node(latched_subscriber)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        latched_publisher.destroy_node()
        latched_subscriber.destroy_node()
        rclpy.shutdown()
```

## Quality of Service (QoS) Profiles

Quality of Service profiles in ROS 2 provide fine-grained control over communication characteristics, which is essential for real-time robotic systems. The four main QoS settings are reliability, durability, history, and liveliness, each addressing different aspects of communication requirements.

Reliability controls whether messages are guaranteed to be delivered. RELIABLE ensures all messages are delivered (with potential delays), while BEST_EFFORT prioritizes timeliness over delivery guarantee. For sensor data like camera feeds or LIDAR scans, BEST_EFFORT is often appropriate since occasional dropped frames are acceptable. For critical control commands, RELIABLE is essential.

Durability determines how messages persist for late-joining subscribers. VOLATILE means messages are only sent to currently connected subscribers, while TRANSIENT_LOCAL means the publisher stores messages and sends them to new subscribers when they join.

History controls how many messages are stored for delivery. KEEP_LAST maintains a fixed-size queue of the most recent messages, while KEEP_ALL stores all messages (with memory implications).

Here's an example of using different QoS profiles for different data types:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class QoSDemoNode(Node):

    def __init__(self):
        super().__init__('qos_demo_node')

        # Sensor data: best-effort, volatile, keep last 10
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.sensor_publisher = self.create_publisher(LaserScan, 'sensor_scan', sensor_qos)

        # Control commands: reliable, volatile, keep last 1
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.control_publisher = self.create_publisher(Twist, 'cmd_vel', control_qos)

        # System status: reliable, transient local, keep last 1
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.status_publisher = self.create_publisher(String, 'system_status', status_qos)

        # Create timers for different types of publications
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)  # 10Hz
        self.control_timer = self.create_timer(0.05, self.publish_control_command)  # 20Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1Hz

    def publish_sensor_data(self):
        # Simulate sensor data publication with best-effort QoS
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        # ... populate laser scan data
        self.sensor_publisher.publish(msg)

    def publish_control_command(self):
        # Simulate control command with reliable QoS
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        msg.angular.z = 0.1  # Turn slightly right
        self.control_publisher.publish(msg)

    def publish_status(self):
        # Publish system status with transient local QoS
        msg = String()
        msg.data = 'System operational - OK'
        self.status_publisher.publish(msg)
```

## Custom Message Types

Custom message types allow you to define application-specific data structures that can be shared between nodes. These are defined in `.msg` files and compiled into language-specific interfaces. Creating well-designed custom messages is crucial for efficient communication in complex robotic systems.

To create a custom message, first create a `msg` directory in your package and define the message structure:

```
# In msg/RobotState.msg
std_msgs/Header header
float64 position_x
float64 position_y
float64 position_z
float64 orientation_x
float64 orientation_y
float64 orientation_z
float64 orientation_w
string status
uint8[] joint_angles
float64[5] sensor_readings
```

Then in your Python code, you can use the custom message:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from your_package_msgs.msg import RobotState  # Custom message type


class RobotStatePublisher(Node):

    def __init__(self):
        super().__init__('robot_state_publisher')
        self.publisher_ = self.create_publisher(RobotState, 'robot_state', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_robot_state)

    def publish_robot_state(self):
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Populate robot state
        msg.position_x = 1.0
        msg.position_y = 2.0
        msg.position_z = 0.0
        msg.orientation_x = 0.0
        msg.orientation_y = 0.0
        msg.orientation_z = 0.0
        msg.orientation_w = 1.0
        msg.status = 'ACTIVE'

        # Joint angles (example: 6 joints)
        msg.joint_angles = [45, 30, 90, 0, -45, 15]  # in degrees

        # Sensor readings (example: 5 sensors)
        msg.sensor_readings = [1.2, 3.4, 5.6, 7.8, 9.0]

        self.publisher_.publish(msg)
```

## Advanced rclpy Features

The rclpy library provides several advanced features beyond basic publishing and subscribing, including parameter callbacks, lifecycle nodes, and custom executors. These features enable more sophisticated node behaviors and system management.

Parameter callbacks allow nodes to react dynamically to parameter changes without restarting:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class DynamicParameterNode(Node):

    def __init__(self):
        super().__init__('dynamic_parameter_node')

        # Declare parameters with default values
        self.declare_parameter('control_gain', 1.0)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('operation_mode', 'manual')

        # Set callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Initialize with current parameter values
        self.control_gain = self.get_parameter('control_gain').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.operation_mode = self.get_parameter('operation_mode').value

        self.get_logger().info(f'Initialized with gain: {self.control_gain}, max vel: {self.max_velocity}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'control_gain':
                self.control_gain = param.value
                self.get_logger().info(f'Control gain updated to: {self.control_gain}')
            elif param.name == 'max_velocity':
                self.max_velocity = param.value
                self.get_logger().info(f'Max velocity updated to: {self.max_velocity}')
            elif param.name == 'operation_mode':
                self.operation_mode = param.value
                self.get_logger().info(f'Operation mode updated to: {self.operation_mode}')

        return SetParametersResult(successful=True)
```

## Timers and Callback Groups

ROS 2 provides advanced timer functionality that allows for precise timing control and coordination between different callbacks. Callback groups enable you to control the threading model of your node, determining which callbacks can run concurrently.

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class AdvancedTimersNode(Node):

    def __init__(self):
        super().__init__('advanced_timers_node')

        # Create different callback groups
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = ReentrantCallbackGroup()  # Can run concurrently with others

        # Create timers with different callback groups
        self.timer1 = self.create_timer(
            0.1,  # 10 Hz
            self.high_freq_callback,
            callback_group=self.group1
        )

        self.timer2 = self.create_timer(
            1.0,  # 1 Hz
            self.low_freq_callback,
            callback_group=self.group2
        )

        self.pub_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publisher_callback,
            callback_group=self.group3
        )

    def high_freq_callback(self):
        self.get_logger().info('High frequency callback executing')

    def low_freq_callback(self):
        self.get_logger().info('Low frequency callback executing')

    def publisher_callback(self):
        # This can run concurrently with other callbacks due to ReentrantCallbackGroup
        self.get_logger().info('Publisher callback executing')
```

## Working with Time and Timeouts

Proper time handling is critical in robotic systems where synchronization between different components is essential. ROS 2 provides sophisticated time management capabilities:

```python
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.clock import Clock, ClockType


class TimeManagementNode(Node):

    def __init__(self):
        super().__init__('time_management_node')

        # Create a clock instance
        self.clock = Clock(clock_type=ClockType.ROS_TIME)

        # Track execution times
        self.last_execution_time = self.clock.now()

        # Create timer for periodic execution
        self.timer = self.create_timer(0.1, self.periodic_task)

    def periodic_task(self):
        current_time = self.clock.now()
        time_since_last = current_time - self.last_execution_time

        # Log execution interval
        self.get_logger().info(f'Task executed after {(time_since_last.nanoseconds / 1e9):.3f}s')

        # Example: timeout logic
        if time_since_last > Duration(seconds=0.2):
            self.get_logger().warn('Execution took longer than expected!')

        # Update last execution time
        self.last_execution_time = current_time

        # Perform actual task
        self.perform_computation()

    def perform_computation(self):
        # Simulate some computation
        import time
        time.sleep(0.01)  # 10ms computation
```

## Conclusion

Advanced ROS 2 communication patterns and rclpy features provide the tools necessary to build sophisticated, reliable robotic systems. Understanding Quality of Service settings, custom message types, parameter management, and timing mechanisms is essential for creating applications that can handle the diverse requirements of real-world robotic systems.

The ability to configure different communication characteristics for different types of data allows you to optimize your robotic system for performance, reliability, and real-time requirements. In the next chapter, we'll explore URDF in greater detail and learn how to create complex robot models for simulation and control.

## Exercises

1. Create a ROS 2 node that publishes sensor data with BEST_EFFORT QoS and another that subscribes with appropriate settings, measuring the actual delivery rate.
2. Implement a custom message type for a mobile robot's odometry that includes position, velocity, and covariance information.
3. Design a parameter configuration system that allows runtime adjustment of PID controller gains.
4. Implement a time-synchronized publisher that ensures messages are published at precise intervals despite variable computation times.