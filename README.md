# Autonomous Mobile Robot (AMR) ROS2 Factory Logistics Simulation

This repository contains a ROS 2 Python package designed for an autonomous differential drive Mobile Robot simulating smart warehouse logistics. It integrates physical simulation in **Gazebo Sim (Harmonic)**, navigation using **Nav2**, precise vision-guided target docking via **QR Code detection**, and real-time **SQLite data logging**.

![Sim Demonstration](https://raw.githubusercontent.com/mohamedazimal27/amr_robot/main/docs/simulation_demo.webp) *(Provide your custom simulation demo video/gif link here)*

---

## 🚀 Key Capabilities & Modules

1. **Physical robot & Gazebo simulation**: Symmetrically balanced 2WD differential drive AMR configured via Xacro/URDF with high-fidelity contact dynamics.
2. **Nav2 autonomous navigation**: Navigates through a cluttered factory map (`opil_factory`) using AMCL localization and costmap path planning.
3. **QR-guided vision alignment**: Real-time vision processor (`qr_detector`) that uses OpenCV to spot stations, filter movements with velocity gates, and publish offsets to adjust docking posture.
4. **Mission orchestration**: Fully autonomous state machine (`mission_manager`) that controls the schedule (Dock ➡️ Station A ➡️ Station B ➡️ Dock) with safety recovery states.
5. **Database logging**: Stores QR detection status, execution metrics, and run cycles in a persistent SQLite database (`maps/inventory.db`) via `inventory_logger`.
6. **Foxglove studio visualization**: Pre-configured visual telemetry layout to monitor robot posture, camera feed overlays, and topic message parameters.

---

## 📐 System Architecture

### Communication Graph (Nodes & Topics)
This diagram illustrates the flow of commands and sensor data between the physical simulation, navigation modules, vision detection, and metrics logging:

```mermaid
graph TD
    %% Core Nodes
    subgraph Navigation Stack
        NAV2["Nav2 Navigation Server"]
        MM["Mission Manager Node"]
    end

    subgraph Vision & Processing
        QR["QR Detector (OpenCV)"]
    end

    subgraph Logging & Analytics
        DB["SQLite database (inventory.db)"]
        LOG["Inventory Logger Node"]
        MET["Metrics Evaluator Node"]
    end

    subgraph Gazebo Simulation
        GZ["Gazebo Harmonic"]
        CAM["Camera sensor"]
        DIFF["Diff Drive Plugin"]
    end

    %% Flow connections
    MM -->|Goal action requests| NAV2
    NAV2 -->|/cmd_vel| DIFF
    DIFF -->|/odom| MM
    CAM -->|/camera/image_raw| QR
    QR -->|/qr_detection_image| RVIZ["Foxglove / RViz2"]
    
    %% Aligning
    QR -->|/detected_qr_raw (alignment offset)| MM
    MM -->|/cmd_vel (alignment tweaks)| DIFF
    
    %% Logs & DB
    MM -->|/log_scan (station scanned event)| LOG
    LOG -->|SQL Writes| DB
    LOG -->|/metrics| MET
```

---

## 📂 Project Directory Structure

```
amr_robot/
├── amr_robot/
│   ├── inventory_logger.py    # Listens to scans and updates the SQLite database
│   ├── metrics_evaluator.py   # Aggregates run times and navigation success rates
│   ├── mission_manager.py     # Main mission state machine and navigation goals
│   └── qr_detector.py         # Image processing & QR detection with OpenCV
├── config/
│   ├── foxglove_layout.json   # Pre-configured Foxglove Studio layout
│   ├── nav2_params.yaml       # Path planning and costmap parameters
│   └── slam_toolbox_params.yaml # Localization configuration
├── launch/
│   ├── gazebo.launch.py       # Boots up the Gazebo simulation and ROS bridges
│   ├── navigation.launch.py   # Starts Nav2 servers with correct map spawn pose
│   └── mission_nodes.launch.py # Starts the vision, logging, and mission nodes
├── maps/
│   ├── inventory.db           # Persistent SQLite database for session tracking
│   ├── opil_factory.pgm       # Warehouse 2D occupancy grid map
│   └── opil_factory.yaml      # Map metadata
├── urdf/
│   └── amr_robot.urdf.xacro   # Symmetrical 2WD URDF description
└── worlds/
    └── factory.world          # Gazebo warehouse environment world
```

---

## 🏁 Quick Start: Running the Autonomous Mission

### 1. Build and Source
Ensure your ROS 2 (Jazzy/Humble) workspace is clean:
```bash
cd ~/ros2_ws
# Build package (do not use --symlink-install to prevent importlib package metadata issues)
colcon build --packages-select amr_robot
source install/setup.bash
```

### 2. Launch Gazebo and Nav2 Stack
Boot up the Gazebo simulator and Nav2 navigation server:
```bash
# Terminal 1: Launch Gazebo
ros2 launch amr_robot gazebo.launch.py

# Terminal 2: Launch Nav2
ros2 launch amr_robot navigation.launch.py map:=opil_factory
```

### 3. Launch the Mission Orchestration Node
Start the QR code detector, SQL logger, metrics aggregator, and mission manager nodes:
```bash
# Terminal 3: Run the Mission Manager
ros2 launch amr_robot mission_nodes.launch.py
```

---

## 📊 Live Monitoring with Foxglove Studio

We provide a custom layout to visualize your AMR metrics. 

1. Install and open **Foxglove Studio**.
2. Connect to your ROS 2 environment (using standard ROS 2 bridge or Rosbridge server).
3. Import the pre-made layout: **`config/foxglove_layout.json`**.
4. You will see:
   - **Left Panel (3D)**: Live occupancy grid map, laser scan footprints, TF frames, and robot trajectory.
   - **Top Right (Image)**: Real-time camera stream showing bounding boxes, centroid trackers, and gating validation (e.g. `VEL_OK`, `CENTER_OK`) in green or red.
   - **Bottom Right (Logs)**: Decoded QR values and alignment offsets published on `/detected_qr_raw`.

---

## 💾 Persistent SQLite Schema

The `inventory_logger` automatically structures the mission logs inside `maps/inventory.db` using this layout:

```sql
CREATE TABLE IF NOT EXISTS scan_logs (
    id INTEGER PRIMARY KEY AUTO_INCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    station_id TEXT NOT NULL,
    status TEXT NOT NULL
);
```

You can view active logs directly in the terminal:
```bash
sqlite3 src/amr_robot/maps/inventory.db "SELECT * FROM scan_logs;"
```

---

## 📐 Map Calibration & Verification

The mission goals use hardcoded coordinates verified directly against map-frame point clouds:
- **Station A**: Approach Pose `(9.2, 11.3)`, Yaw `-1.176 rad` (-67.4°)
- **Station B**: Approach Pose `(5.2, 17.6)`, Yaw `2.069 rad` (118.5°)
- **Dock (Home)**: Spawn Pose `(18.91, 11.724)`, Yaw `-1.876 rad`
