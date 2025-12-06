---
sidebar_position: 3
---

# URDF: Unified Robot Description Format for Physical AI Systems

## Learning Objectives

By the end of this chapter, you will be able to:
- Create complex robot models using URDF with multiple links, joints, and visual elements
- Implement collision detection and inertial properties for accurate simulation
- Integrate URDF models with ROS 2 for robot state publishing and TF transformations
- Use Xacro macros to simplify complex robot descriptions
- Design robot models that work effectively in both simulation and real-world applications

## Real-World Context

URDF (Unified Robot Description Format) is fundamental to robotics development as it provides a standardized way to describe robot geometry, kinematics, and dynamics. Modern robotics systems, from industrial manipulators to humanoid robots, rely on accurate URDF models for simulation, visualization, motion planning, and control. In the context of Physical AI and Humanoid Robotics, URDF enables researchers and engineers to create digital twins of their robots that can be tested in simulation before deployment on physical hardware.

Companies like Boston Dynamics, Tesla, and numerous robotics startups use URDF or similar formats to model their robots. For humanoid robots specifically, accurate URDF models are essential for complex tasks like walking, grasping, and manipulation that require precise understanding of the robot's physical structure and capabilities.

## Understanding URDF Structure

URDF is an XML-based format that describes a robot's physical and visual properties. The core elements include links (rigid bodies), joints (constraints between links), and additional elements for visual representation, collision detection, and inertial properties.

**Figure: URDF hierarchical structure showing robot tree with base link and connected joints** - This diagram illustrates the tree structure of a URDF model with a base_link at the root, connected to various child links through different joint types. Each link has associated visual, collision, and inertial properties, with coordinate frames attached to each link following the right-hand rule.

A robot in URDF is represented as a tree structure of links connected by joints. The tree must have a single root link (the base link), and each joint connects exactly two links - a parent and a child. This tree structure represents the kinematic chain of the robot, defining how different parts move relative to each other.

The basic URDF structure includes:

- **Links**: Represent rigid bodies with visual, collision, and inertial properties
- **Joints**: Define how links connect and move relative to each other
- **Materials**: Define visual appearance properties
- **Transmissions**: Define how actuators connect to joints (for control)
- **Gazebo tags**: Define simulation-specific properties

Here's an example of a more complex URDF structure:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.15"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="8.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.12"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.12"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting base to torso -->
  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting torso to head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <inertial>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

</robot>
```

## Joint Types and Their Applications

URDF supports several joint types, each serving different kinematic purposes:

- **Fixed**: No movement between links (welded connection)
- **Revolute**: Rotational movement around a single axis with limits
- **Continuous**: Unlimited rotational movement around a single axis
- **Prismatic**: Linear sliding movement along a single axis with limits
- **Floating**: Six degrees of freedom (x, y, z, roll, pitch, yaw)
- **Planar**: Movement in a plane with rotation around normal axis

Different joint types are appropriate for different robot components. Revolute joints are common for most robotic arms and legs, while prismatic joints might be used for telescoping mechanisms. Continuous joints are used when unlimited rotation is needed, such as in wheels or rotating sensors.

## Visual and Collision Properties

Visual elements define how the robot appears in simulation and visualization tools, while collision elements define the geometry used for physics simulation and collision detection. These can be different shapes to balance visual quality with computational efficiency.

For example, a complex visual mesh might use thousands of polygons for realistic appearance, while the collision geometry might use simpler primitive shapes like boxes, cylinders, and spheres that are more efficient for physics calculations.

## Inertial Properties and Dynamics

Accurate inertial properties are crucial for realistic simulation and proper control of dynamic systems. Each link must specify its mass, center of mass, and moment of inertia tensor. These properties determine how the robot responds to forces and torques in simulation.

The moment of inertia tensor is a 3x3 matrix that describes how mass is distributed in a body relative to its axes. In URDF, only the diagonal elements (ixx, iyy, izz) and the off-diagonal elements (ixy, ixz, iyz) are specified.

## Working with Xacro Macros

Xacro (XML Macros) extends URDF by providing macros, constants, and mathematical expressions to simplify complex robot descriptions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot_xacro">

  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_radius" value="0.15" />
  <xacro:property name="base_height" value="0.2" />
  <xacro:property name="torso_height" value="0.4" />
  <xacro:property name="arm_length" value="0.2" />

  <!-- Macro for creating a simple cylindrical link -->
  <xacro:macro name="cylinder_link" params="name radius length mass color xyz_rpy:=0 0 0 0 0 0">
    <link name="${name}">
      <inertial>
        <origin xyz="${xyz_rpy[0]} ${xyz_rpy[1]} ${xyz_rpy[2]}" rpy="${xyz_rpy[3]} ${xyz_rpy[4]} ${xyz_rpy[5]}"/>
        <mass value="${mass}"/>
        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
      </inertial>
      <visual>
        <origin xyz="${xyz_rpy[0]} ${xyz_rpy[1]} ${xyz_rpy[2]}" rpy="${xyz_rpy[3]} ${xyz_rpy[4]} ${xyz_rpy[5]}"/>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      <collision>
        <origin xyz="${xyz_rpy[0]} ${xyz_rpy[1]} ${xyz_rpy[2]}" rpy="${xyz_rpy[3]} ${xyz_rpy[4]} ${xyz_rpy[5]}"/>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Macro for creating a revolute joint -->
  <xacro:macro name="revolute_joint" params="name parent child origin_xyz origin_rpy axis_xyz lower upper effort velocity">
    <joint name="${name}" type="revolute">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
      <axis xyz="${axis_xyz}"/>
      <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
    </joint>
  </xacro:macro>

  <!-- Base link using macro -->
  <cylinder_link name="base_link" radius="${base_radius}" length="${base_height}" mass="10.0" color="blue"/>

  <!-- Torso using macro -->
  <cylinder_link name="torso" radius="0.12" length="${torso_height}" mass="8.0" color="orange" xyz_rpy="0 0 ${base_height/2 + torso_height/2} 0 0 0"/>

  <!-- Joint connecting base to torso -->
  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 ${base_height/2 + torso_height/2}" rpy="0 0 0"/>
  </joint>

  <!-- Head using macro -->
  <link name="head">
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Neck joint -->
  <revolute_joint name="neck_joint" parent="torso" child="head"
                  origin_xyz="0 0 ${torso_height/2 + 0.05}"
                  origin_rpy="0 0 0"
                  axis_xyz="0 1 0"
                  lower="-0.5" upper="0.5" effort="100" velocity="1"/>

</robot>
```

## Integration with ROS 2

URDF models integrate with ROS 2 through several mechanisms:

1. **Robot State Publisher**: Publishes TF transforms for all links in the robot
2. **Joint State Publisher**: Publishes joint positions for visualization
3. **TF2**: Provides transformation services between coordinate frames
4. **Motion Planning**: Provides kinematic and dynamic information for planners

To use URDF with ROS 2, you typically create a launch file that starts the robot state publisher:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF file path
    urdf_dir = get_package_share_directory('your_robot_description')
    urdf_path = os.path.join(urdf_dir, 'urdf', 'humanoid_robot.urdf.xacro')

    # Read URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Joint State Publisher node (for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher,
        joint_state_publisher
    ])
```

## Advanced URDF Features

URDF also supports more advanced features for complex robotic systems:

**Transmissions**: Define how actuators connect to joints for control:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

**Gazebo-specific tags**: Define simulation-specific properties:

```xml
<gazebo reference="left_upper_arm">
  <material>Gazebo/Green</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

## Best Practices for URDF Modeling

When creating URDF models for Physical AI and Humanoid Robotics, consider these best practices:

1. **Accuracy**: Ensure dimensions and masses match the physical robot as closely as possible
2. **Simplicity**: Use simple geometric shapes for collision detection while using detailed meshes for visualization
3. **Hierarchy**: Design a logical tree structure that reflects the robot's kinematic chains
4. **Units**: Always use meters for distances and kilograms for mass
5. **Frame conventions**: Follow ROS coordinate frame conventions (X-forward, Y-left, Z-up)
6. **Joint limits**: Always specify realistic joint limits to prevent damage in simulation
7. **Mass properties**: Calculate or estimate mass properties as accurately as possible

## Conclusion

URDF is a critical component of the ROS ecosystem, providing the standardized format for describing robot models that enables simulation, visualization, motion planning, and control. Understanding how to create accurate and efficient URDF models is essential for developing Physical AI and Humanoid Robotics applications.

Well-designed URDF models enable realistic simulation, proper motion planning, and accurate control of robotic systems. In the next chapter, we'll explore how to use these models in simulation environments like Gazebo to test and validate our robotic applications.

## Exercises

1. Create a URDF model for a simple 6-DOF robotic arm with proper inertial properties and joint limits.
2. Implement a Xacro macro for a commonly repeated component in your robot design.
3. Create a launch file that loads your URDF model and publishes the robot state.
4. Design a humanoid leg with hip, knee, and ankle joints, ensuring proper kinematic chain structure.