# Autonomous Mobile Robot (AMR) ROS2 Simulation

A ROS2 (Robot Operating System 2) Python package containing the URDF (Unified Robot Description Format) description and simulation environment for a 2WD differential drive Autonomous Mobile Robot (AMR). The project is integrated with Gazebo Sim (Harmonic) and includes sensors such as Lidar, IMU, and a Camera.

## 🚀 Features

- **URDF Description**: Fully defined differential drive robot description utilizing Xacro, featuring links for a base, deck, caster, wheels, Lidar, and camera.
- **Gazebo Integration**: Pre-configured plugins for the differential drive control system (`gz-sim-diff-drive-system`) and joint state publishing (`gz-sim-joint-state-publisher-system`).
- **Sensors**: Integrated Lidar sensor model broadcasting on `/scan` with a custom range of `0.12` to `12.0` meters and standard Gaussian noise.
- **Factory World**: A simulation world environment loaded from `worlds/factory.world`.
- **Launch Files**: Ready-to-go Python launch files for booting up the robot state publisher, spawning the robot in Gazebo, and bridging the topics.

## 📂 Project Structure

```
amr_robot/
├── amr_robot/                 # Python source files/nodes
│   └── __init__.py
├── config/
│   └── slam_toolbox_params.yaml # Config parameters for SLAM
├── launch/
│   ├── gazebo.launch.py       # Main launch file to spawn robot and run bridge
│   └── slam.launch.py         # SLAM launch configuration
├── resource/
│   └── amr_robot              # Resource registration marker
├── test/                      # Code quality tests (flake8, copyright, pep257)
├── urdf/
│   └── amr_robot.urdf.xacro   # Robot Xacro physical description and gazebo properties
├── worlds/
│   └── factory.world          # Factory environment world simulation
├── package.xml                # ROS 2 package manifest
├── setup.cfg                  # Python packaging configuration
└── setup.py                   # Setup script for Python package installer
```

## 🛠️ Prerequisites

Make sure you have the following installed on your system:
- **ROS2** (Jazzy / Humble)
- **Gazebo** (Harmonic / Garden)
- **ROS-Gazebo Bridges**: `ros_gz_sim` and `ros_gz_bridge`

## 🏁 How to Run

### 1. Build and Source Workspace

From the root of your ROS2 workspace (e.g., `~/ros2_ws`):

```bash
# Build the package
colcon build --packages-select amr_robot

# Source the overlay
source install/setup.bash
```

### 2. Launch Simulation

To launch the robot state publisher, open Gazebo Harmonic in the factory world, spawn the robot at `(0.0, -2.5, 0.15)`, and launch the bidirectional ROS-Gazebo bridges:

```bash
ros2 launch amr_robot gazebo.launch.py
```

### 3. Topics Bridged

The simulation automatically bridges the following topics between ROS2 and Gazebo:
- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/odom` (`nav_msgs/msg/Odometry`)
- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/tf` (`tf2_msgs/msg/TFMessage`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/clock` (`rosgraph_msgs/msg/Clock`)
