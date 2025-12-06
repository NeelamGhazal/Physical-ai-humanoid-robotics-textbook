---
sidebar_position: 1
---

# Gazebo/Unity: Simulation Fundamentals for Physical AI Systems

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the role of simulation in Physical AI and robotics development
- Create and configure physics-based simulation environments using Gazebo
- Implement sensor integration and realistic physics parameters
- Design simulation scenarios for testing and validation
- Bridge simulation to real-world robotics using ROS 2 integration

## Real-World Context

Simulation is a cornerstone of modern robotics development, providing a safe, cost-effective, and efficient environment for testing algorithms, training AI models, and validating robot behaviors before deployment to physical hardware. In the field of Physical AI and Humanoid Robotics, simulation environments like Gazebo play a crucial role in developing complex behaviors that would be difficult, expensive, or dangerous to test on real robots.

Leading robotics companies and research institutions use simulation extensively. For example, companies developing humanoid robots use simulation to test walking gaits, balance control, and manipulation tasks before transferring these behaviors to physical robots. This approach, known as "sim-to-real transfer," has proven essential for developing sophisticated robotic behaviors efficiently.

## Introduction to Physics-Based Simulation

Physics-based simulation in robotics involves modeling the physical laws that govern how objects interact in the real world. This includes Newtonian mechanics, friction, collisions, and other physical phenomena that affect robot behavior. Accurate physics simulation is essential for developing controllers that can work effectively in the real world.

**Figure: Physics simulation comparison showing real robot vs simulated robot performing identical task** - This diagram illustrates how a humanoid robot in simulation performs the same walking gait as its physical counterpart, with physics parameters tuned to match real-world behavior. The simulation environment includes gravity, friction, and collision properties that mirror the real world.

Gazebo provides a sophisticated physics engine that models these interactions accurately. The physics engine calculates forces, torques, collisions, and resulting motions for all objects in the simulation environment. This enables developers to test complex robotic behaviors in a virtual environment that closely approximates real-world physics.

The simulation process involves several key components:

1. **World Definition**: Creating the environment with terrain, objects, and physics properties
2. **Robot Model**: Loading the URDF model with accurate physical properties
3. **Physics Engine**: Computing interactions based on physical laws
4. **Sensor Simulation**: Modeling how sensors would perceive the simulated environment
5. **Control Interface**: Connecting the simulation to ROS 2 for control and data exchange

## Gazebo Architecture and Components

Gazebo is built around a modular architecture that allows for flexible simulation scenarios. The core components include:

- **Physics Engine**: Handles collision detection and response using ODE, Bullet, or Simbody
- **Rendering Engine**: Provides visual representation using OGRE
- **Sensors**: Simulates various sensor types (cameras, LIDAR, IMUs, etc.)
- **Communication Interface**: Connects to ROS/ROS 2 via Gazebo-ROS packages

The Gazebo simulation loop operates at a fixed rate, typically 1000 Hz for physics calculations and a lower rate for rendering. During each iteration, the physics engine computes forces and updates positions, sensors generate data, and control commands are processed.

Here's an example of a basic Gazebo world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define a simple box obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
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
            <specular>1 0 0 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

## Sensor Integration in Simulation

Sensors are critical components in robotics simulation, as they provide the robot with information about its environment. Gazebo provides realistic simulation of various sensor types including cameras, LIDAR, IMUs, force/torque sensors, GPS, and more.

Camera sensors in Gazebo simulate pinhole camera models with realistic noise and distortion parameters:

```xml
<sensor name="camera" type="camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="head">
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>800</width>
      <height>600</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_frame</frame_name>
    <topic_name>camera/image_raw</topic_name>
  </plugin>
</sensor>
```

LIDAR sensors simulate laser range finders with configurable parameters:

```xml
<sensor name="lidar" type="ray">
  <pose>0 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="laserscan" filename="libgazebo_ros_laser.so">
    <topic_name>scan</topic_name>
    <frame_name>lidar_frame</frame_name>
  </plugin>
</sensor>
```

## Physics Parameters and Tuning

Accurate physics parameters are crucial for realistic simulation and successful sim-to-real transfer. Key parameters include:

- **Gravity**: Typically set to Earth's gravity (-9.8 m/s² in the z-direction)
- **Friction**: Defines how objects interact when in contact (static and dynamic coefficients)
- **Damping**: Controls energy loss in joints and motion
- **Material properties**: Density, elasticity, and other material characteristics

Properly tuning these parameters requires careful comparison between simulated and real-world robot behavior. Techniques like system identification can be used to determine accurate physical parameters for real robots, which can then be applied to simulation models.

## Unity as an Alternative Simulation Environment

While Gazebo is the traditional choice for ROS-based robotics simulation, Unity has emerged as a powerful alternative, particularly for applications requiring high-fidelity graphics and complex environments. Unity's physics engine (NVIDIA PhysX) and rendering capabilities make it attractive for certain types of robotics applications, especially those involving computer vision and human-robot interaction.

Unity robotics simulation typically involves:

- **Unity ML-Agents**: For reinforcement learning and training
- **Unity Robotics Package**: For ROS communication
- **High-fidelity rendering**: For photorealistic simulation
- **Complex environments**: For testing in detailed virtual worlds

Unity provides advantages in scenarios where visual realism is critical, such as training computer vision models or testing human-robot interaction scenarios. However, Gazebo remains the standard for traditional robotics applications due to its tight integration with ROS and extensive sensor simulation capabilities.

## ROS 2 Integration with Gazebo

The integration between ROS 2 and Gazebo is facilitated by the Gazebo-ROS packages, which provide bridges between Gazebo's native communication system and ROS 2 topics and services. This integration enables seamless control of simulated robots using ROS 2 nodes.

Here's an example of launching a robot in Gazebo with ROS 2 integration:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='basic_world')

    # Get paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('robot_description')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': os.path.join(pkg_robot_description, 'worlds', 'basic_world.sdf'),
            'verbose': 'false',
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'world_name',
            default_value='basic_world',
            description='Choose one of the world files from `/robot_description/worlds`'),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

## Creating Custom Simulation Environments

Developing custom simulation environments involves several steps:

1. **Define the world**: Create SDF files that define the environment geometry, physics properties, and objects
2. **Model obstacles and fixtures**: Add static and dynamic objects that represent the operating environment
3. **Configure sensors**: Set up cameras, LIDAR, and other sensors to match real-world configurations
4. **Parameterize physics**: Adjust friction, damping, and other parameters to match real-world conditions
5. **Test scenarios**: Develop specific scenarios to test robot behaviors

For humanoid robotics applications, simulation environments often include:

- **Terrain variations**: Different surfaces to test locomotion algorithms
- **Obstacles**: Objects to test navigation and manipulation capabilities
- **Dynamic elements**: Moving objects to test reactive behaviors
- **Interactive objects**: Items that respond to robot actions

## Simulation Testing Strategies

Effective simulation testing involves multiple strategies:

- **Unit Testing in Simulation**: Test individual components in isolation
- **Integration Testing**: Test how multiple subsystems work together
- **Regression Testing**: Ensure new changes don't break existing functionality
- **Edge Case Testing**: Test boundary conditions and failure scenarios
- **Performance Testing**: Evaluate computational efficiency and real-time performance

Simulation testing allows for accelerated development cycles since tests can run much faster than real-time and can be automated extensively. This is particularly valuable for training machine learning models that require thousands of trials.

## Bridging Simulation to Reality

The ultimate goal of simulation is to develop behaviors and algorithms that work effectively on real robots. This "sim-to-real" transfer requires careful attention to:

- **Model fidelity**: Ensuring simulation parameters match real-world values
- **Domain randomization**: Training with varied simulation parameters to improve robustness
- **System identification**: Measuring real robot parameters to tune simulation
- **Control adaptation**: Adjusting control parameters between simulation and reality

Successful sim-to-real transfer has been demonstrated in numerous applications, from quadruped locomotion to manipulation tasks, making simulation an indispensable tool in modern robotics development.

## Conclusion

Simulation environments like Gazebo and Unity provide essential capabilities for developing Physical AI and Humanoid Robotics applications. Understanding how to create realistic simulation scenarios with accurate physics, proper sensor modeling, and effective ROS 2 integration is crucial for efficient robotics development.

The ability to test and validate robotic behaviors in simulation before deployment to physical hardware saves time, reduces costs, and minimizes risks. In the next chapter, we'll explore how to use these simulation environments for specific robotics applications and advanced testing scenarios.

## Exercises

1. Create a Gazebo world with multiple obstacles and test robot navigation through it.
2. Implement a camera sensor in simulation and verify that it publishes images to ROS 2 topics.
3. Tune physics parameters for a simple robot to match its real-world behavior.
4. Design a simulation scenario for testing humanoid robot walking gaits.