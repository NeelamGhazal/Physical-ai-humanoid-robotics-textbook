---
sidebar_position: 1
---

# Simulation Basics: Gazebo and Unity for Physical AI Systems

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the fundamental role of simulation in Physical AI and robotics development
- Compare Gazebo and Unity simulation platforms for different robotics applications
- Set up and configure basic simulation environments with both platforms
- Identify appropriate use cases for each simulation platform
- Integrate simulation environments with ROS 2 for robotics development

## Real-World Context

Simulation plays a crucial role in robotics development, allowing us to test algorithms, train AI models, and validate designs in a safe, controlled environment before deploying to physical robots. In the field of Physical AI and Humanoid Robotics, simulation environments are essential for developing complex behaviors that would be dangerous, expensive, or time-consuming to test on real hardware.

Major robotics companies like Boston Dynamics, Tesla, and countless research institutions use simulation extensively to develop and validate their robotic systems. For humanoid robots specifically, simulation allows for testing of walking gaits, balance control, and manipulation tasks before transferring these behaviors to expensive physical platforms. This approach, known as "sim-to-real transfer," has proven essential for developing sophisticated robotic behaviors efficiently.

Modern robotics development pipelines heavily rely on simulation for training machine learning models, especially for computer vision and reinforcement learning applications. Synthetic data generated from high-fidelity simulations can be used to pre-train models that are later fine-tuned on real-world data, significantly reducing the amount of physical testing required.

**Figure: Simulation-to-Reality Pipeline showing development process from simulation to real robot deployment** - This diagram illustrates how robotics development typically begins in simulation where algorithms are tested and refined, then moves to sim-to-real transfer where models are adapted for physical hardware, and finally to real-world deployment where the robot operates in actual environments. The pipeline shows feedback loops where real-world performance can inform simulation improvements.

## Why Simulate?

Simulation offers several critical advantages in robotics development:

- **Safety**: Test dangerous maneuvers, failure scenarios, and emergency responses without risk to expensive hardware or humans
- **Cost-effectiveness**: Reduce wear and tear on physical robots, minimize hardware damage during development, and reduce operational costs
- **Speed**: Run experiments faster than real-time, accelerate training of machine learning models, and iterate rapidly through design cycles
- **Repeatability**: Exact same conditions for testing, enabling controlled experiments and reliable validation of improvements
- **Scalability**: Test on multiple simulated robots simultaneously, evaluate multi-robot scenarios, and parallelize experiments
- **Environment Control**: Create controlled scenarios with known ground truth, test edge cases, and reproduce specific conditions reliably
- **Data Generation**: Generate large datasets for training AI models with perfect annotations and ground truth data

## Gazebo: Physics-Based Simulation

Gazebo is a physics-based simulation environment that provides realistic sensor simulation and contact dynamics. Originally developed for the ROS ecosystem, Gazebo has become the de facto standard for robotics simulation in academic and industrial settings.

### Key Features

- **Physics Engine**: Multiple options including ODE (Open Dynamics Engine), Bullet Physics, and SimBody for realistic physics simulation
- **Sensor Simulation**: Comprehensive support for cameras, LIDAR, IMUs, GPS, force/torque sensors, and custom sensors
- **Plugin Architecture**: Extensible with custom plugins for specialized behaviors and ROS integration
- **ROS Integration**: Native support for ROS/ROS2 with Gazebo-ROS packages for seamless communication
- **Open Source**: Free and open-source with active community development and support

### Basic Gazebo Concepts

- **Worlds**: Environments defined in SDF (Simulation Description Format) containing physics properties, lighting, and objects
- **Models**: Robots, obstacles, and objects in the simulation defined in SDF or URDF formats
- **SDF**: Simulation Description Format, an XML-based format for defining simulation elements
- **Plugins**: Dynamic libraries that extend Gazebo's functionality for custom behaviors and interfaces
- **Services**: Gazebo provides various services for controlling simulation state, spawning models, and setting parameters

### Advanced Gazebo Capabilities

Gazebo supports complex scenarios including multi-robot simulation, dynamic environments, and realistic sensor noise modeling. The physics engine accurately simulates contact forces, friction, and collisions, making it ideal for testing control algorithms that depend on accurate force feedback.

Here's an example of creating a custom Gazebo plugin for ROS 2 integration:

```xml
<!-- Example SDF model with ROS 2 plugin -->
<sdf version="1.7">
  <model name="custom_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </visual>
      <sensor name="camera" type="camera">
        <camera name="head">
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <frame_name>camera_frame</frame_name>
          <topic_name>camera/image_raw</topic_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
```

## Unity: Game Engine for Robotics

Unity provides a high-fidelity graphics engine that's increasingly used for robotics simulation, particularly for computer vision applications and human-robot interaction studies. While not originally designed for robotics, Unity's powerful rendering capabilities and physics engine make it attractive for certain robotics applications.

### Key Features

- **High-Fidelity Graphics**: Photo-realistic rendering with advanced lighting, shadows, and materials using physically-based rendering
- **XR Support**: Native support for virtual and augmented reality applications
- **Asset Store**: Extensive marketplace with pre-built environments, robots, and objects
- **Cross-Platform**: Deploy to multiple platforms including Windows, macOS, Linux, and mobile devices
- **Scripting**: Flexible C# scripting environment for custom behaviors and logic
- **Machine Learning**: Built-in ML-Agents framework for reinforcement learning applications

### Unity Robotics Integration

Unity has developed specific tools for robotics applications:

- **Unity Robotics Hub**: Centralized package manager for robotics-related packages
- **ROS#**: Package for ROS/ROS 2 communication with Unity
- **ML-Agents**: Framework for training intelligent agents using reinforcement learning
- **Unity Perception**: Tools for generating synthetic training data with ground truth annotations

### Unity vs Traditional Robotics Simulation

Unity excels in scenarios requiring high-quality graphics and realistic visual rendering. This makes it particularly valuable for:
- Training computer vision models with photorealistic synthetic data
- Testing visual SLAM algorithms
- Human-robot interaction studies
- Virtual reality teleoperation interfaces
- Public demonstrations and visualization

However, Unity's physics engine, while capable, is generally not as accurate as specialized robotics simulators like Gazebo for precise force control and contact dynamics.

## Comparison: Gazebo vs Unity

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| Physics | Excellent (ODE, Bullet, SimBody) | Good (NVIDIA PhysX) |
| Graphics | Basic | Excellent (RTX, PBR) |
| Sensor Simulation | Comprehensive | Limited (but extensible) |
| Learning Curve | Moderate | Steeper (C# and Unity concepts) |
| Community | Robotics-focused | Gaming-focused with growing robotics adoption |
| Cost | Free (open source) | Free (personal), Paid (professional) |
| ROS Integration | Native, seamless | Requires ROS# package |
| Performance | Optimized for robotics | Optimized for graphics |
| Use Cases | Control, navigation, physics | Vision, interaction, VR/AR |

## Setting Up Your First Simulation

### Gazebo Example

To get started with Gazebo, you can launch basic simulations and begin experimenting:

```bash
# Launch empty world
gz sim -r -v 4 empty.sdf

# Launch with GUI for visualization
gz sim -g -r -v 4 empty.sdf

# Launch with a simple robot model
gz sim -g -r -v 4 warehouse.sdf

# Create a custom launch file for your robot
# This example shows how to integrate with ROS 2
```

### Gazebo with ROS 2 Integration

For robotics development, Gazebo is typically used with ROS 2 through the Gazebo-ROS packages:

```python
# Example launch file for Gazebo simulation with ROS 2
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('robot_description')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': os.path.join(pkg_robot_description, 'worlds', 'empty.sdf'),
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gazebo
    ])
```

### Unity Example

Unity requires the Unity Editor and additional packages for robotics-specific features:

1. Install Unity Hub and a recent version of Unity (2021.3 LTS or newer)
2. Install the Unity Robotics Simulation package
3. Add the ROS# package for ROS/ROS 2 communication
4. Import robot models and environment assets
5. Configure sensors and control interfaces

Unity's workflow typically involves:
- Designing scenes in the Unity Editor
- Configuring sensors and physics properties
- Writing C# scripts for robot control and ROS communication
- Building and running simulations

## Advanced Simulation Techniques

### Domain Randomization

To improve sim-to-real transfer, domain randomization techniques are used to train models across a wide variety of simulated conditions:

- Randomizing lighting conditions
- Varying material properties and textures
- Adding visual noise and artifacts
- Changing physics parameters within realistic bounds

### Synthetic Data Generation

Simulation environments can generate large amounts of labeled training data with perfect annotations:

- Semantic segmentation masks
- Depth maps
- Object bounding boxes
- 3D pose information
- Optical flow fields

## Practical Exercise: Basic Simulation Setup

1. Install Gazebo Garden (or latest version) on your development machine
2. Launch an empty world using `gz sim -g -r -v 4 empty.sdf`
3. Spawn a simple robot model such as the RRBot (Revolute-Revolute Manipulator)
4. Control the robot joints using ROS 2 commands to observe basic movement
5. Add sensors to the robot and visualize the sensor data
6. Create a simple world file with obstacles and test navigation

## Real-World Applications

Simulation environments are used across numerous robotics applications:

- **Training computer vision models**: Generating synthetic datasets with perfect annotations
- **Testing navigation algorithms**: Validating path planning and obstacle avoidance
- **Human-robot interaction studies**: Developing intuitive interfaces and behaviors
- **Multi-robot coordination**: Testing swarm behaviors and communication protocols
- **Reinforcement learning for robotics**: Training policies in safe, controllable environments
- **Hardware validation**: Testing new sensors and actuators before physical integration
- **Safety validation**: Testing failure modes and emergency procedures without risk

## Best Practices for Simulation Development

1. **Validate Simulation Accuracy**: Compare simulation results with real-world data to ensure fidelity
2. **Model Real-World Imperfections**: Include sensor noise, actuator delays, and environmental variations
3. **Use Appropriate Fidelity**: Balance simulation accuracy with computational efficiency
4. **Document Assumptions**: Clearly document the limitations and assumptions of your simulation
5. **Plan for Sim-to-Real Transfer**: Design simulations with real-world deployment in mind

## Summary

Simulation environments like Gazebo and Unity provide essential capabilities for developing Physical AI and Humanoid Robotics applications. Gazebo excels in physics accuracy and ROS integration, making it ideal for control, navigation, and physics-based applications. Unity provides superior graphics and is excellent for computer vision and human-robot interaction applications.

Understanding when to use each platform and how to configure them appropriately is crucial for effective robotics development. The choice between them often depends on the specific requirements of your application, with many projects benefiting from using both platforms for different aspects of development.

The integration of simulation with real robotics development workflows enables faster, safer, and more cost-effective development of complex robotic systems. As robotics continues to advance, simulation will remain a cornerstone of the development process.

## Exercises

1. Install both Gazebo and Unity and run basic simulation examples with each platform
2. Compare the physics simulation quality between Gazebo and Unity using a simple falling object test
3. Create a simple world file in Gazebo with a robot and obstacles, then test navigation
4. Set up a basic Unity scene with a robot model and configure basic movement controls
5. Evaluate which simulation platform is more appropriate for different robotics applications based on specific requirements