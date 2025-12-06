/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2',
      items: [
        'ros2/introduction',
        'ros2/topics-rclpy',
        'ros2/urdf-modelling'
      ],
    },
    {
      type: 'category',
      label: 'Gazebo & Unity',
      items: [
        'gazebo-unity/simulation-basics',
        'gazebo-unity/simulation-fundamentals',
        'gazebo-unity/physics-sensors'
      ],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac',
      items: [
        'nvidia-isaac/robotics-framework',
        'nvidia-isaac/introduction',
        'nvidia-isaac/vslam-nav2'
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA)',
      items: [
        'vla/introduction'
      ],
    },
  ],
};

export default sidebars;