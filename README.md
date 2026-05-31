# Autonomous Mobile Robot (AMR) ROS2 Simulation

A ROS2 (Robot Operating System 2) Python package containing the URDF (Unified Robot Description Format) description and simulation environment for a 2WD differential drive Autonomous Mobile Robot (AMR). The project is integrated with Gazebo Sim (Harmonic) and includes sensors such as Lidar, IMU, and a Camera.

> [!NOTE]
> For a highly detailed explanation of the coordinate frame tree, system node architecture, physical enhancements (like caster wheels and joint state publishers), and lifecycle SLAM synchronization details, please refer to the dedicated [Mapping & Simulation Guide](MAPPING_AND_SIMULATION.md).

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
│   ├── nav2_params.yaml       # Config parameters for Nav2
│   └── slam_toolbox_params.yaml # Config parameters for SLAM
├── launch/
│   ├── gazebo.launch.py       # Main launch file to spawn robot and run bridge
│   ├── navigation.launch.py   # Nav2 autonomous navigation launch routine
│   └── slam.launch.py         # SLAM launch configuration
├── maps/
│   ├── demo_factory.pgm       # Generated 2D warehouse map image
│   └── demo_factory.yaml      # Generated map metadata
├── resource/
│   └── amr_robot              # Resource registration marker
├── rviz/
│   └── nav2.rviz              # Pre-configured RViz 2 dashboard for Nav2
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

### 2. Launch Simulation & RViz Visualization

To boot up the simulation, launch the robot state publisher, open Gazebo Harmonic in the factory world, spawn the robot at `(0.0, -2.5, 0.15)`, and start the bidirectional ROS-Gazebo bridges:

```bash
ros2 launch amr_robot gazebo.launch.py
```

### 3. Topics Bridged

The simulation automatically bridges the following active topics between ROS2 and Gazebo:
- `/cmd_vel` (`geometry_msgs/msg/Twist`) — Bidirectional teleoperation control.
- `/odom` (`nav_msgs/msg/Odometry`) — Odometry state from Gazebo.
- `/scan` (`sensor_msgs/msg/LaserScan`) — Lidar range scan data for SLAM.
- `/tf` (`tf2_msgs/msg/TFMessage`) — Coordinate transforms.
- `/joint_states` (`sensor_msgs/msg/JointState`) — Wheel rotational states from the simulation.
- `/clock` (`rosgraph_msgs/msg/Clock`) — Synced simulation clock time.

---

## 🗺️ Mapping the Environment (SLAM)

This package integrates `slam_toolbox` (in synchronized mode) to build maps of the simulation world.

### 1. Launch the SLAM Node & RViz
Once the Gazebo simulation is running, launch the SLAM mapping toolbox and RViz in a new terminal:

```bash
# Source workspace
source install/setup.bash

# Launch SLAM nodes & RViz
ros2 launch amr_robot slam.launch.py
```
- In RViz, set the **Fixed Frame** to `map`.
- The SLAM node runs as a ROS 2 lifecycle node and automatically configures and activates itself to establish the `map -> odom` transform.

### 2. Drive the Robot (Teleoperation)
Open a third terminal and run the keyboard teleop node to steer the robot around the factory:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the standard keyboard inputs (`i` to drive forward, `,` to drive backward, `j`/`l` to steer left/right) to navigate the factory floor and map all walls and obstacles.

### 3. Save the Map
When you are satisfied with the built map in RViz, save it by calling the `SaveMap` service (which expects a nested `std_msgs/String name` layout):

```bash
# Create directory for maps
mkdir -p src/amr_robot/maps

# Save map files (maps/demo_factory.pgm & maps/demo_factory.yaml)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/mohamed-azimal/ros2_ws/src/amr_robot/maps/demo_factory'}}"
```

---

## 🛠️ Enhancements & Fixes Implemented

We have implemented key upgrades to the robot description and simulation setup to ensure high-fidelity physical realism and visualization:

1. **Dual Caster Symmetrical Balance**:
   - Added a symmetrical **front caster wheel** and renamed the rear caster to `rear_caster_wheel`.
   - Set contact parameters and zero friction coefficients (`mu1 = 0.0`, `mu2 = 0.0`) in Gazebo to prevent dragging, tipping, or gravity slanting.
2. **Corrected Active Wheel Joints**:
   - Inverted wheel joint axes from `<axis xyz="0 0 1"/>` to `<axis xyz="0 0 -1"/>` in the URDF, fixing direction control so the robot moves forward with `i` and backward with `,`.
3. **Smooth RViz Wheel Rotation (No Jitter)**:
   - Added the `gz-sim-joint-state-publisher-system` plugin to publish simulated joint states on `/joint_states`.
   - Removed the conflicting static `joint_state_publisher` ROS 2 node from the launch routine, resulting in perfectly smooth wheel rotation in RViz.
4. **Lifecycle-Managed SLAM Node**:
   - Integrated the official `online_sync_launch.py` template inside `slam.launch.py` to ensure proper lifecycle transition states (`configure` and `activate`) are triggered automatically. This guarantees the `/map -> /odom` frame path exists immediately in the coordinate tree.
