Here is the **complete technical project plan** for the Autonomous Inventory Scanning Bot, simulation-only, built on your **ROS 2 Jazzy + Gazebo Harmonic** stack. No code—just architecture, timelines, technology decisions, and deliverables.

---

# Project: Autonomous Inventory Scanning Bot (Simulation)

**Stack:** ROS 2 Jazzy Jalisco | Gazebo Harmonic | Ubuntu 24.04  
**Duration:** 6 Weeks | **Effort:** ~15 hrs/week  
**Output:** GitHub portfolio + simulation demo video + metrics report

---

## 1. System Architecture

### 1.1 High-Level Data Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                         GAZEBO HARMONIC                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐  │
│  │  Warehouse   │  │  AMR Bot     │  │  Inventory Stations      │  │
│  │  World (SDF) │  │  + LiDAR     │  │  (QR-coded panels)       │  │
│  │  + Shelves   │  │  + RGB Cam   │  │  + Dock Station          │  │
│  │  + Lighting  │  │  + IMU       │  │                          │  │
│  └──────────────┘  └──────────────┘  └──────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      ROS_GZ BRIDGE (Parameter Bridge)                │
│  Translates gz.msgs.*  ↔  ros_msgs.* (LaserScan, Image, Imu, Odom)   │
└─────────────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌──────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Nav2 Stack │    │  SLAM Toolbox   │    │  QR Detector    │
│   (Jazzy)    │    │  (Async Online) │    │  Node           │
│  • AMCL      │    │                 │    │  • OpenCV       │
│  • Planner   │    │                 │    │  • pyzbar       │
│  • Controller│    │                 │    │  • Center-gate  │
│  • BT Navigator│   │                 │    │  • Confidence   │
└──────────────┘    └─────────────────┘    └─────────────────┘
        │                     │                     │
        └─────────────────────┼─────────────────────┘
                              ▼
              ┌─────────────────────────────┐
              │      Mission Manager Node    │
              │  • State Machine (INIT→NAV→  │
              │    SCAN→LOG→NEXT→DOCK)      │
              │  • Nav2 Simple Commander API │
              │  • Retry logic (1 nudge)     │
              └─────────────────────────────┘
                              │
                              ▼
              ┌─────────────────────────────┐
              │    Inventory Logger Node     │
              │  • SQLite3 Database          │
              │  • Schema: timestamp, station│
              │    qr_content, pose, status  │
              └─────────────────────────────┘
                              │
                              ▼
              ┌─────────────────────────────┐
              │   Metrics Evaluator (CLI)    │
              │  • Ground truth comparison   │
              │  • Success rate calculation  │
              │  • Markdown report export    │
              └─────────────────────────────┘
```

### 1.2 ROS 2 Graph Structure

| Node Name | Inputs | Outputs | Responsibility |
|-----------|--------|---------|----------------|
| `gz_sim` | — | gz topics | Physics, sensor simulation |
| `ros_gz_bridge` | gz topics | ROS topics | Protocol translation |
| `robot_state_publisher` | `/joint_states` | `/tf`, `/tf_static` | Kinematic tree |
| `slam_toolbox` | `/scan`, `/tf` | `/map`, `/posegraph` | Occupancy grid mapping |
| `amcl` | `/scan`, `/map`, `/tf` | `/amcl_pose` | Monte Carlo localization |
| `planner_server` | `/amcl_pose`, goal | `/plan` | Global path (Smac Hybrid-A*) |
| `controller_server` | `/plan`, `/scan` | `/cmd_vel` | Local trajectory (Regulated Pure Pursuit) |
| `qr_detector` | `/camera/image_raw` | `/detected_qr`, `/qr_detection_image` | Vision pipeline |
| `mission_manager` | `/detected_qr`, nav status | goals to Nav2, scan commands | Orchestration |
| `inventory_logger` | `/detected_qr`, mission events | SQLite writes | Persistence |
| `metrics_evaluator` | SQLite DB, ground truth | stdout report | Validation |

---

## 2. Phase-by-Phase Execution Plan

### Phase 1: Simulation Environment & Robot Model
**Timeline:** Week 1  
**Goal:** A reproducible, version-controlled Gazebo world with a sensor-rich robot.

#### 1.1 World Design Specifications
- **Dimensions:** 12m × 8m warehouse floor
- **Aisle Layout:** 4 parallel shelving rows, 1.2m aisle width (constrains navigation, proves planner skill)
- **Materials:** Concrete floor texture (visual odometry stability), diffuse metal shelving
- **Lighting:** 4 overhead point lights with shadows enabled (critical for camera realism)
- **Dynamic Elements:** None (static world for baseline; optional: add 1 dynamic box in Phase 6 as stretch)

#### 1.2 Inventory Stations
- **Count:** 4 stations
- **Placement:** Mounted on shelving units at 0.4m height, facing the aisle centerline
- **QR Content:** `STATION-A-001`, `STATION-A-002`, `STATION-B-001`, `STATION-B-002`
- **Physical Form:** 15cm × 15cm flat panels with textured QR codes (SDF material with albedo map)

#### 1.3 Dock Station
- **Location:** Origin corner (0, 0)
- **Visual Marker:** Green floor patch + small charging station model
- **Function:** Mission start and end pose

#### 1.4 Robot Kinematics & Sensors
- **Chassis:** Differential drive, 0.25m wheelbase, 0.05m wheel radius
- **LiDAR:** 360-sample, 10Hz, 0.12m–10m range, mounted at 0.2m height
- **Camera:** RGB, 640×480, 30FPS, mounted at 0.3m height, 15° downward pitch
- **IMU:** 6-axis, 50Hz, mounted at chassis center
- **Odometry:** Derived from diff-drive plugin (ground truth equivalent for simulation)

#### 1.5 Deliverables
- `warehouse.sdf` — version-controlled world file
- `bot.urdf.xacro` — parameterized robot description
- `bridge.yaml` — `ros_gz` parameter bridge configuration
- `sim.launch.py` — single-command launch bringing up Gazebo + bridge + robot state publisher
- **Validation:** Manual teleop drive; confirm all topics echo data; RViz shows TF tree and laser scan

---

### Phase 2: Mapping & Localization
**Timeline:** Week 2  
**Goal:** Persistent warehouse map and reliable autonomous localization.

#### 2.1 SLAM Configuration
- **Tool:** SLAM Toolbox (async online mode)
- **Parameters:**
  - Resolution: 0.05m
  - Max laser range: 10.0m
  - Minimum travel distance: 0.5m (new node insertion)
  - Loop closure: enabled
  - Scan matching: Ceres-based
- **Process:** Manual teleop exploration of all aisles and stations; save serialized pose graph + PGM/YAML map pair
- **Output:** `warehouse_map.yaml` + `warehouse_map.pgm` + `warehouse_map.posegraph`

#### 2.2 Localization (AMCL)
- **Tool:** Nav2 AMCL
- **Configuration:**
  - Particle count: 2000
  - Laser min range: 0.12m
  - Update min distance: 0.2m
  - Update min angle: 0.25 rad
- **Validation:** Drive robot to 4 known waypoints; compare AMCL pose to Gazebo ground truth; drift must be < 5cm

#### 2.3 Navigation Stack (Nav2)
- **Global Planner:** Smac Hybrid-A* (handles narrow aisles better than NavFn; proves advanced Nav2 knowledge)
- **Local Controller:** Regulated Pure Pursuit (curvature regulation, safer in corridors)
- **Behavior Tree:** `navigate_to_pose_w_replanning_and_recovery.xml` (default with recovery behaviors)
- **Recovery Actions:** Spin, backup, clear costmap
- **Costmap:** 2-layer (static map + inflation + obstacle from laser)

#### 2.4 Deliverables
- `nav2_params.yaml` — fully tuned parameter file
- `localization.launch.py` — AMCL + map server
- `navigation.launch.py` — full Nav2 bringup
- **Validation:** Send 2D Nav Goals via RViz to all 4 station coordinates; robot must reach each without collision

---

### Phase 3: Perception Pipeline
**Timeline:** Week 3  
**Goal:** Reliable QR detection with false-positive rejection.

#### 3.1 QR Code Generation
- **Library:** Python `qrcode` module
- **Format:** PNG, 21×21 modules (Version 1), high error correction
- **Naming Convention:** Matches station IDs exactly (enables ground-truth validation)

#### 3.2 Detection Pipeline Specifications
- **Input Topic:** `/camera/image_raw`
- **Processing Rate:** 10Hz (throttled to reduce CPU)
- **Detection Library:** pyzbar (wraps ZBar)
- **Center-Gate Logic:** Only accept QR if barcode centroid falls within ±20% of image center (prevents side-aisle false reads)
- **Velocity-Gate Logic:** Only accept QR if robot linear velocity < 0.05 m/s (prevents motion blur reads)
- **Output Topics:**
  - `/detected_qr` — `std_msgs/String` with decoded content
  - `/qr_detection_image` — annotated image with bounding boxes for RViz

#### 3.3 Visualization Requirements
- RViz panel showing live camera feed
- Overlay topic showing green bounding boxes + decoded text
- Text marker in 3D view showing last successful scan at robot location

#### 3.4 Deliverables
- `qr_detector` ROS package (node specification, launch file, parameter YAML)
- QR texture assets in `warehouse_gazebo/models/`
- **Validation:** Place robot at 0.5m, 1.0m, 1.5m from station; measure detection rate at each distance; optimal range is 0.6m–1.2m

---

### Phase 4: Mission Orchestration & Data Logging
**Timeline:** Week 4  
**Goal:** End-to-end autonomous mission with persistent data layer.

#### 4.1 Mission State Machine
**States:**
1. `INIT` — Wait for Nav2 active, clear costmaps
2. `LOCALIZE` — Initial pose estimate (manual or auto)
3. `NAV_TO_STATION` — Send goal to Nav2; monitor status
4. `SCAN` — Halt motion, enable QR detection for 3 seconds
5. `VERIFY` — Compare detected QR to expected station ID
6. `LOG` — Write to SQLite regardless of match
7. `RETRY` — If mismatch or timeout, nudge forward 0.1m, rescan once
8. `NEXT` — Increment station index
9. `RETURN_TO_DOCK` — Navigate to origin pose
10. `DONE` — Publish mission summary, shutdown sequence

#### 4.2 Navigation Integration
- **API:** `nav2_simple_commander` Python API (BasicNavigator)
- **Goal Tolerance:** 0.15m position, 0.1 rad yaw
- **Timeout per leg:** 60 seconds (triggers recovery)
- **Replanning:** Enabled every 2 seconds if obstacle detected

#### 4.3 Database Schema
**Table:** `scans`

| Column | Type | Description |
|--------|------|-------------|
| `id` | INTEGER PRIMARY KEY | Auto-increment |
| `timestamp` | TEXT (ISO 8601) | Scan completion time |
| `station_id` | TEXT | Expected station (ground truth) |
| `qr_content` | TEXT | Detected QR payload |
| `robot_pose_x` | REAL | AMCL X coordinate at scan time |
| `robot_pose_y` | REAL | AMCL Y coordinate at scan time |
| `robot_pose_yaw` | REAL | AMCL yaw at scan time |
| `status` | TEXT | `SUCCESS` (match), `MISMATCH` (wrong QR), `TIMEOUT` (no read) |

#### 4.4 Deliverables
- `mission_manager` package (state machine node, parameter config)
- `inventory_logger` package (SQLite interface node)
- `mission.launch.py` — brings up QR detector + mission manager + logger
- **Validation:** Run full mission manually triggered; verify DB entries match expected 4 rows with correct statuses

---

### Phase 5: Metrics, Ground Truth & Evaluation
**Timeline:** Week 5  
**Goal:** Quantifiable proof of performance.

#### 5.1 Ground Truth System
- **Source:** Gazebo Harmonic `PosePublisher` plugin on robot model
- **Bridge:** `ros_gz_bridge` mapping `/model/bot/pose` → `/ground_truth`
- **Frequency:** 50Hz
- **Use:** Post-hoc comparison against AMCL poses logged in SQLite

#### 5.2 Evaluation Metrics

| Metric | Definition | Target | Measurement Method |
|--------|-----------|--------|------------------|
| Navigation Success Rate | Fraction of station goals reached | 100% | Nav2 goal status callback |
| Localization Drift | Euclidean error AMCL vs. ground truth | < 5cm | Post-process ground truth topic against DB pose |
| QR Detection Accuracy | Fraction of correct scans | ≥ 95% | DB `status='SUCCESS'` / total stations |
| False Positive Rate | Incorrect QR reads while moving | 0% | Audit `/detected_qr` during transit phases |
| Average Scan Time | Time from arrival to verified scan | < 8s | Timestamp delta in DB |
| Mission Completion Time | Total dock-to-dock duration | < 5 min | First to last DB timestamp |
| Recovery Triggers | How many times Nav2 recovery activates | < 2 | Behavior tree log parsing |

#### 5.3 Report Generator
- **Tool:** Python CLI script
- **Input:** SQLite DB + ROS bag (optional) + ground truth export
- **Output:** Markdown report (`mission_report.md`) with:
  - Mission summary table
  - Per-station breakdown
  - Trajectory accuracy plot (matplotlib, saved as PNG)
  - Metric dashboard

#### 5.4 Deliverables
- `metrics_evaluator` package
- `generate_report.py` CLI tool
- Sample report from a successful mission run
- **Validation:** Run 3 full missions back-to-back; metrics must be consistent across runs

---

### Phase 6: Integration, Polish & Documentation
**Timeline:** Week 6  
**Goal:** Portfolio-ready, recruiter-facing deliverable.

#### 6.1 Master Launch System
**Single-command launch:** `full_mission.launch.py`
- **Sequence:** Gazebo → Bridge → Robot State Publisher → Nav2 (with map) → QR Detector → Mission Manager → Logger → RViz
- **Parameterization:** Launch arguments for `use_rviz`, `use_slam` (true for mapping, false for nav), `map_file`

#### 6.2 RViz Configuration
- **Panels Required:**
  - Map display
  - Robot model
  - Laser scan (decay time: 5s)
  - Global plan (green)
  - Local plan (blue)
  - Camera image (raw + annotated overlay)
  - Marker array (scan locations, last QR text)
  - TF tree (frames: `map`, `odom`, `base_link`, `lidar_link`, `camera_link`)

#### 6.3 Repository Structure
```
inventory-bot-sim/
├── README.md                 # Project overview, stack, GIF demo
├── docs/
│   ├── SETUP.md              # Jazzy + Harmonic install steps
│   ├── ARCHITECTURE.md       # Node graph, topic list, TF tree
│   ├── NAV2_TUNING.md        # Why Smac Hybrid-A* vs. NavFn
│   └── METRICS.md            # Benchmark results from 3 runs
├── src/
│   ├── warehouse_description/   # URDF, Xacro, meshes
│   ├── warehouse_gazebo/        # SDF world, QR station models
│   ├── qr_detector/             # Node spec, params, assets
│   ├── mission_manager/         # State machine spec, params
│   ├── inventory_logger/        # DB schema, node spec
│   └── metrics_evaluator/       # Report generator spec
├── maps/
│   ├── warehouse_map.yaml
│   ├── warehouse_map.pgm
│   └── warehouse_map.posegraph
├── config/
│   ├── bridge.yaml
│   ├── nav2_params.yaml
│   ├── rviz_config.rviz
│   └── mission_params.yaml
└── scripts/
    ├── install_deps.sh
    ├── generate_qr_textures.py
    └── run_evaluation.sh
```

#### 6.4 Demo Video Specifications
- **Length:** 60–90 seconds
- **Segments:**
  1. Gazebo warehouse view (0:00–0:10) — show scale and stations
  2. RViz view (0:10–0:30) — map, laser, plan, camera overlay
  3. Terminal split-screen (0:30–0:50) — live QR detection logs + DB writes
  4. Final report (0:50–1:00) — metrics dashboard
- **Format:** GIF for GitHub README, MP4 for LinkedIn/portfolio

#### 6.5 README Writing Guide
**Sections:**
- **Problem Statement:** "Warehouse inventory audits are manual and slow. This project simulates an AMR solution."
- **Technical Stack:** Bullet list with versions (Jazzy, Harmonic, Nav2, etc.)
- **Architecture:** Embed the data flow diagram
- **Key Results:** Table with your actual metrics from 3 runs
- **What I Learned:** 3 bullet points showing growth (e.g., "Tuned Smac Hybrid-A* for narrow aisles," "Built full sensor-to-database pipeline")
- **Run It Yourself:** 3 commands to replicate

---

## 3. Technology Justifications

| Decision | Why This Choice |
|----------|----------------|
| **Jazzy over Humble** | Current LTS; Harmonic native support; shows you work with modern stacks |
| **Gazebo Harmonic** | Official Jazzy partner; `ros_gz` bridge is stable; SDF 1.10 features |
| **Smac Hybrid-A*** | Handles holonomic constraints in narrow aisles; proves advanced Nav2 knowledge beyond default NavFn |
| **Regulated Pure Pursuit** | Curvature regulation prevents wall clipping; industry-preferred for AMRs over DWB for simple chassis |
| **SLAM Toolbox (async)** | Online mapping without bag files; faster iteration than Cartographer for this scale |
| **SQLite over ROS bags** | Persistent, queryable, zero-config; shows data engineering awareness; recruiters can read the DB |
| **pyzbar over custom CV** | Robust, proven; focus project on systems integration, not reinventing QR detection |
| **Center + Velocity gates** | Simulates real-world gating logic; demonstrates you think about false positives, not just happy-path detection |

---

## 4. Risk Register & Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Nav2 fails in narrow aisles | Medium | High | Switch to Smac Hybrid-A*; increase inflation layer; reduce robot footprint in URDF |
| QR detection fails at angle | Medium | High | Tune camera pitch to 15°; place stations perpendicular to aisle; add 1 retry nudge |
| Gazebo Harmonic TF issues | Low | High | Explicit `<gz_frame_id>` in every SDF sensor; verify with `ros2 topic echo /tf` before any nav |
| Jazzy package availability | Low | Medium | Use `ros-jazzy-desktop-full` install; verify `ros-jazzy-nav2-bringup` exists before starting |
| Simulation runs too slow | Medium | Medium | Reduce camera resolution to 320×240 for dev; increase to 640×480 only for final demo |
| Mission manager hangs on Nav2 | Medium | High | Implement 60s timeout per leg; add `cancelTask()` on timeout; log state transitions |

---

## 5. Weekly Time Breakdown

| Week | Primary Focus | Approx. Hours | Key Milestone |
|------|--------------|---------------|---------------|
| 1 | World + Robot + Bridge | 12–15 | `sim.launch.py` runs; all sensors publish |
| 2 | SLAM + Nav2 | 15–18 | Saved map; RViz nav goals succeed to all stations |
| 3 | QR Assets + Vision | 12–15 | Detection rate ≥ 90% at 0.8m in static test |
| 4 | Mission Logic + DB | 15–18 | Full mission runs; DB populates 4 rows |
| 5 | Metrics + Ground Truth | 12–15 | 3 mission runs; metrics stable; report generates |
| 6 | Integration + Docs | 12–15 | Single launch; README + video complete |

---

## 6. Recruiter-Facing Outcomes

After completing this plan, you can claim these **specific, verifiable** competencies:

- **"ROS 2 Jazzy ecosystem"** — launch systems, parameter handling, node composition, `ros_gz` bridging
- **"Nav2 production configuration"** — Smac Hybrid-A* global planning, Regulated Pure Pursuit local control, AMCL tuning, behavior tree customization
- **"SLAM & localization"** — SLAM Toolbox mapping, persistent map server, pose estimation validation
- **"Perception pipeline design"** — camera-to-vision integration, false-positive gating, annotation overlays
- **"Autonomous mission planning"** — state machines, failure recovery, retry logic, timeout handling
- **"Data pipeline engineering"** — SQLite persistence, schema design, automated reporting, metric extraction
- **"Simulation-to-reality methodology"** — Gazebo Harmonic sensor modeling, ground truth validation, reproducible environments

---

## 7. Post-Project Stretch Goals (If Time Permits)

1. **Dynamic Obstacle Avoidance:** Add a human-shaped model walking across an aisle; verify replanning
2. **Multi-Floor Map:** Use Gazebo levels to simulate a mezzanine; test map switching
3. **Web Dashboard:** Flask app reading SQLite via REST API; live mission monitor
4. **Behavior Tree Custom:** Write a custom BT node for "scan until QR found or timeout"
5. **CI/CD:** GitHub Actions running `colcon test` and linting on every push

---

**Final Instruction:** Do not start Phase 2 until Phase 1 validation passes. Do not start Phase 4 until Phase 3 detection rate exceeds 90% in static tests. The most common failure mode in robotics portfolio projects is rushing to mission logic before the underlying navigation and perception are independently reliable.