---
sidebar_position: 2
---

# Physics Simulation and Sensor Integration in Gazebo/Unity

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure realistic physics parameters for accurate simulation of robotic systems
- Implement and calibrate various sensor types in simulation environments
- Model environmental factors like friction, gravity, and material properties
- Design sensor fusion algorithms that work in both simulation and reality
- Validate simulation accuracy against real-world robot behavior

## Real-World Context

Physics simulation and sensor integration form the backbone of effective robotic development workflows. Modern robotics applications require accurate modeling of physical interactions to ensure that algorithms developed in simulation can successfully transfer to real-world robots. This is particularly important in Physical AI and Humanoid Robotics, where robots must interact with complex physical environments.

Companies developing humanoid robots invest heavily in simulation environments that accurately model real-world physics. For example, when developing bipedal walking algorithms, the simulation must accurately model ground reaction forces, friction coefficients, and dynamic stability to ensure that gaits learned in simulation will work on physical robots.

## Physics Engine Configuration

Gazebo provides access to multiple physics engines, each with different strengths and characteristics. The most commonly used engines are:

- **ODE (Open Dynamics Engine)**: Default engine, good for general-purpose simulation
- **Bullet**: Good performance with robust collision detection
- **Simbody**: Advanced multi-body dynamics with constraints

The physics engine configuration affects how accurately the simulation models real-world physics. Key parameters include:

- **Gravity**: Typically set to Earth's gravity (9.8 m/s²) unless simulating other celestial bodies
- **Max Step Size**: Defines the maximum time step for physics calculations (smaller = more accurate but slower)
- **Real Time Factor**: Controls how fast the simulation runs compared to real time
- **Solver Iterations**: Number of iterations for constraint solving (more = more stable but slower)

**Figure: Physics parameter comparison showing simulation vs real robot behavior** - This diagram illustrates how adjusting physics parameters like damping, friction, and solver iterations can bring simulated robot behavior closer to real-world performance. The graph shows position trajectories of a simulated joint versus its real-world counterpart before and after parameter tuning.

Here's an example of configuring physics parameters in a Gazebo world file:

```xml
<physics type="ode">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>1000</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Material Properties and Friction Modeling

Accurate modeling of material properties is crucial for realistic simulation. Friction coefficients determine how objects interact when in contact, affecting everything from walking gaits to manipulation tasks. The two main friction parameters are:

- **Static friction**: Force required to initiate motion between surfaces
- **Dynamic friction**: Force required to maintain motion between surfaces

Different materials have different friction properties. For example, rubber on concrete has high friction, while ice on metal has low friction. In simulation, these properties must be carefully tuned to match real-world behavior.

```xml
<material name="rubber">
  <script>
    <uri>file://media/materials/scripts/gazebo.material</uri>
    <name>Gazebo/Black</name>
  </script>
</material>

<!-- Friction properties in collision elements -->
<link name="wheel">
  <collision name="collision">
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>  <!-- Static friction coefficient -->
          <mu2>1.0</mu2>  <!-- Dynamic friction coefficient -->
          <slip1>0.0</slip1>
          <slip2>0.0</slip2>
        </ode>
        <torsional>
          <coefficient>1.0</coefficient>
          <use_patch_radius>false</use_patch_radius>
          <surface_radius>0.01</surface_radius>
        </torsional>
      </friction>
      <contact>
        <ode>
          <soft_cfm>0.0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>1e+10</kp>
          <kd>1.0</kd>
          <max_vel>100.0</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

## Sensor Implementation and Calibration

Sensors in simulation must be calibrated to match their real-world counterparts. This involves configuring parameters like noise characteristics, update rates, and measurement ranges. Proper calibration is essential for successful sim-to-real transfer.

### Camera Sensors

Camera sensors in Gazebo simulate pinhole camera models with realistic noise and distortion characteristics:

```xml
<sensor name="rgb_camera" type="camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="front_camera">
    <pose>0.1 0 0.1 0 0 0</pose>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_optical_frame</frame_name>
    <topic_name>camera/image_raw</topic_name>
    <hack_baseline>0.07</hack_baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
  </plugin>
</sensor>
```

### LIDAR Sensors

LIDAR sensors require careful configuration of angular resolution, range, and noise parameters:

```xml
<sensor name="3d_lidar" type="gpu_lidar">
  <pose>0.1 0 0.2 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>32</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>
        <max_angle>0.261799</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_gpu_laser.so">
    <topic_name>laser_scan</topic_name>
    <frame_name>lidar_frame</frame_name>
  </plugin>
</sensor>
```

### IMU Sensors

IMU sensors provide crucial information about robot orientation and acceleration:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0.1 0 0 0</pose>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <topic_name>imu/data</topic_name>
    <body_name>imu_link</body_name>
    <frame_name>imu_frame</frame_name>
    <gaussian_noise>0.001</gaussian_noise>
    <update_rate>100</update_rate>
  </plugin>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Environmental Factors Modeling

Realistic simulation requires modeling various environmental factors that affect robot behavior:

### Gravity and Atmospheric Conditions

While Earth's gravity is the default, simulation environments can model different gravitational conditions for applications like space robotics. Atmospheric conditions, though often simplified in simulation, can affect sensors like barometric pressure sensors.

### Lighting and Visual Conditions

For computer vision applications, lighting conditions significantly affect sensor performance. Gazebo allows for detailed lighting configuration:

```xml
<light name="directional_light" type="directional">
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
```

### Terrain and Surface Properties

Different terrains affect robot mobility differently. Modeling these properties accurately is essential for locomotion applications:

```xml
<model name="rough_terrain">
  <link name="terrain_link">
    <collision name="collision">
      <geometry>
        <mesh>
          <uri>model://rough_terrain/meshes/terrain.stl</uri>
        </mesh>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model://rough_terrain/meshes/terrain.stl</uri>
        </mesh>
      </geometry>
    </visual>
  </link>
</model>
```

## Sensor Fusion in Simulation

Modern robotics applications often combine data from multiple sensors to improve perception accuracy. Simulation provides a controlled environment for developing and testing sensor fusion algorithms:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R


class SensorFusionNode(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribers for different sensor types
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/laser_scan', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)

        # Publisher for fused state estimate
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/fused_pose', 10)

        # Internal state variables
        self.current_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.position_estimate = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.velocity_estimate = np.array([0.0, 0.0, 0.0])  # vx, vy, vz

        # Covariance matrices for uncertainty tracking
        self.orientation_covariance = np.eye(3) * 0.1
        self.position_covariance = np.eye(3) * 0.1

        # Kalman filter parameters
        self.process_noise = np.eye(6) * 0.01
        self.measurement_noise = np.eye(6) * 0.1

    def imu_callback(self, msg):
        """Process IMU data for orientation estimation"""
        # Extract orientation from IMU
        self.current_orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # Update orientation covariance based on IMU noise characteristics
        self.orientation_covariance = np.diag([
            msg.orientation_covariance[0],
            msg.orientation_covariance[4],
            msg.orientation_covariance[8]
        ])

        # Process angular velocity for velocity estimation
        angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Integrate to estimate pose change (simplified)
        dt = 0.01  # Assuming 100Hz IMU
        self.integrate_angular_velocity(angular_vel, dt)

    def lidar_callback(self, msg):
        """Process LIDAR data for position estimation"""
        # Extract relevant measurements from laser scan
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter out invalid measurements
        valid_indices = (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        # Convert to Cartesian coordinates
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)

        # Use landmark-based positioning if known landmarks exist
        # This is a simplified example - real implementation would be more complex
        if len(x_points) > 0:
            # Estimate position based on nearest obstacle
            min_idx = np.argmin(valid_ranges)
            closest_range = valid_ranges[min_idx]
            closest_angle = valid_angles[min_idx]

            # Update position estimate with uncertainty
            self.update_position_estimate(
                closest_range, closest_angle, msg.range_min, msg.range_max)

    def camera_callback(self, msg):
        """Process camera data for visual odometry"""
        # This would typically involve feature detection and tracking
        # For simulation, we'll just acknowledge the image reception
        self.get_logger().info(f'Camera image received with dimensions: {msg.width}x{msg.height}')

    def integrate_angular_velocity(self, angular_vel, dt):
        """Integrate angular velocity to update orientation"""
        # Convert angular velocity to quaternion derivative
        omega_quat = np.array([0.0, angular_vel[0], angular_vel[1], angular_vel[2]])
        quat_derivative = self.quaternion_multiply(omega_quat, self.current_orientation) * 0.5

        # Integrate to update orientation
        new_orientation = self.current_orientation + quat_derivative * dt
        # Normalize quaternion
        self.current_orientation = new_orientation / np.linalg.norm(new_orientation)

    def update_position_estimate(self, range_val, angle, min_range, max_range):
        """Update position estimate based on LIDAR measurements"""
        # Simplified position update - real implementation would use more sophisticated methods
        if min_range < range_val < max_range:
            # Estimate position based on known landmark positions
            # This is a placeholder for more complex SLAM algorithms
            pass

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    def publish_fused_estimate(self):
        """Publish the fused pose estimate"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set position
        pose_msg.pose.pose.position.x = self.position_estimate[0]
        pose_msg.pose.pose.position.y = self.position_estimate[1]
        pose_msg.pose.pose.position.z = self.position_estimate[2]

        # Set orientation
        pose_msg.pose.pose.orientation.x = self.current_orientation[0]
        pose_msg.pose.pose.orientation.y = self.current_orientation[1]
        pose_msg.pose.pose.orientation.z = self.current_orientation[2]
        pose_msg.pose.pose.orientation.w = self.current_orientation[3]

        # Set covariance
        pose_msg.pose.covariance = np.block([
            [self.position_covariance, np.zeros((3, 3))],
            [np.zeros((3, 3)), self.orientation_covariance]
        ]).flatten()

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Simulation Validation Techniques

Validating simulation accuracy is crucial for ensuring effective sim-to-real transfer. Several techniques can be employed:

### System Identification

Compare real robot responses to known inputs with simulated responses to tune model parameters:

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


def system_identification(real_data, sim_data, initial_params):
    """
    Optimize simulation parameters to match real robot behavior
    """
    def objective(params):
        # Update simulation with new parameters
        sim_response = simulate_with_params(sim_data['inputs'], params)

        # Calculate error between real and simulated responses
        error = np.mean((real_data['outputs'] - sim_response)**2)
        return error

    # Optimize parameters
    result = minimize(objective, initial_params, method='BFGS')
    return result.x


def compare_trajectories(real_traj, sim_traj, threshold=0.05):
    """
    Compare real and simulated trajectories to validate simulation accuracy
    """
    # Calculate RMSE between trajectories
    rmse = np.sqrt(np.mean((real_traj - sim_traj)**2))

    # Check if error is within acceptable bounds
    is_valid = rmse < threshold

    return {
        'rmse': rmse,
        'is_valid': is_valid,
        'threshold': threshold,
        'difference': real_traj - sim_traj
    }
```

## Unity-Specific Simulation Features

When using Unity for robotics simulation, additional considerations apply:

- **Physics Engine**: Unity uses NVIDIA PhysX, which has different characteristics than Gazebo's engines
- **Graphics Pipeline**: Unity's rendering pipeline enables photorealistic simulation
- **ML-Agents**: Unity's machine learning framework for training AI agents
- **RosSharp**: Package for ROS/ROS 2 communication

Unity excels in scenarios requiring high-fidelity graphics, such as computer vision training or human-robot interaction studies. The ability to create photorealistic environments makes it ideal for training neural networks that need to handle diverse visual conditions.

## Conclusion

Physics simulation and sensor integration are fundamental to developing effective robotic systems. Proper configuration of physics parameters, sensor models, and environmental conditions enables realistic simulation that facilitates successful sim-to-real transfer. The ability to validate simulation accuracy against real-world robot behavior ensures that algorithms developed in simulation will work effectively on physical hardware.

Understanding these concepts is essential for anyone working in Physical AI and Humanoid Robotics, as simulation provides a safe, cost-effective environment for testing complex behaviors before deployment to expensive hardware.

## Exercises

1. Configure a simulation environment with accurate friction properties for a wheeled robot and validate its motion characteristics.
2. Implement a sensor fusion algorithm that combines IMU, LIDAR, and camera data in simulation.
3. Calibrate camera parameters in simulation to match a real camera's intrinsic and extrinsic parameters.
4. Design and implement a system identification experiment to tune simulation parameters based on real robot data.