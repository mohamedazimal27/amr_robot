#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import sqlite3
import json
import os
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime

class MetricsEvaluator(Node):
    def __init__(self):
        super().__init__('metrics_evaluator')
        
        # Declare DB path parameter
        self.declare_parameter('db_path', '/home/mohamed-azimal/ros2_ws/src/amr_robot/maps/inventory.db')
        self.db_path = os.path.expanduser(self.get_parameter('db_path').value)
        
        # Output directory for reports
        self.declare_parameter('output_dir', '/home/mohamed-azimal/ros2_ws/src/amr_robot/docs')
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.init_db()
        
        # Latest received ground truth pose
        self.latest_gt_pose = None
        
        # Subscribers
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        
        self.gt_sub = self.create_subscription(
            PoseStamped,
            '/ground_truth',
            self.gt_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            String,
            '/log_scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info("Metrics Evaluator node initialized and waiting for messages.")
        
    def init_db(self):
        db_dir = os.path.dirname(self.db_path)
        os.makedirs(db_dir, exist_ok=True)
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Ensure scans table exists so we can safely delete from it
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS scans (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    station_id TEXT NOT NULL,
                    qr_content TEXT,
                    robot_pose_x REAL,
                    robot_pose_y REAL,
                    robot_pose_yaw REAL,
                    status TEXT NOT NULL
                )
            ''')
            
            # Ensure trajectory table exists
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS trajectory (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    amcl_x REAL,
                    amcl_y REAL,
                    amcl_yaw REAL,
                    gt_x REAL,
                    gt_y REAL,
                    gt_yaw REAL,
                    drift REAL
                )
            ''')
            
            # Clear previous runs
            cursor.execute("DELETE FROM trajectory")
            cursor.execute("DELETE FROM scans")
            
            conn.commit()
            conn.close()
            self.get_logger().info("SQLite Database tables initialized and cleared for new run.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize SQLite Database: {e}")
            
    def gt_callback(self, msg):
        self.latest_gt_pose = msg.pose
        
    def amcl_callback(self, msg):
        if self.latest_gt_pose is None:
            return
            
        amcl_x = msg.pose.pose.position.x
        amcl_y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        amcl_yaw = math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))
        
        gt_x = self.latest_gt_pose.position.x
        gt_y = self.latest_gt_pose.position.y
        gt_z = self.latest_gt_pose.orientation.z
        gt_w = self.latest_gt_pose.orientation.w
        gt_yaw = math.atan2(2.0 * (gt_w * gt_z), 1.0 - 2.0 * (gt_z * gt_z))
        
        drift = math.sqrt((amcl_x - gt_x)**2 + (amcl_y - gt_y)**2)
        
        timestamp = datetime.utcnow().isoformat()
        
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute('''
                INSERT INTO trajectory (timestamp, amcl_x, amcl_y, amcl_yaw, gt_x, gt_y, gt_yaw, drift)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ''', (timestamp, amcl_x, amcl_y, amcl_yaw, gt_x, gt_y, gt_yaw, drift))
            conn.commit()
            conn.close()
        except Exception as e:
            self.get_logger().error(f"Failed to insert trajectory point: {e}")
            
    def scan_callback(self, msg):
        try:
            data = json.loads(msg.data)
            station_id = data.get('station_id')
            status = data.get('status')
            
            if station_id == 'dock' and status == 'COMPLETE':
                self.get_logger().info("Mission COMPLETE detected. Generating final reports...")
                # Add a short delay to ensure final log gets committed
                time_sleep_ok = False
                try:
                    import time
                    time.sleep(1.0)
                    time_sleep_ok = True
                except:
                    pass
                self.generate_reports()
        except Exception as e:
            self.get_logger().error(f"Error handling scan log in metrics evaluator: {e}")
            
    def generate_reports(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute("SELECT amcl_x, amcl_y, gt_x, gt_y, drift FROM trajectory")
            traj_data = cursor.fetchall()
            
            cursor.execute("SELECT timestamp, station_id, qr_content, status FROM scans")
            scan_data = cursor.fetchall()
            
            conn.close()
            
            if not traj_data:
                self.get_logger().warn("No trajectory data found in database. Cannot generate report.")
                return
                
            amcl_xs = [pt[0] for pt in traj_data]
            amcl_ys = [pt[1] for pt in traj_data]
            gt_xs = [pt[2] for pt in traj_data]
            gt_ys = [pt[3] for pt in traj_data]
            drifts = [pt[4] for pt in traj_data]
            
            mean_drift = sum(drifts) / len(drifts)
            max_drift = max(drifts)
            final_drift = drifts[-1]
            
            # 1. Plot Trajectory
            plt.figure(figsize=(10, 8))
            plt.plot(gt_xs, gt_ys, 'g-', label='Ground Truth (Gazebo)', linewidth=2)
            plt.plot(amcl_xs, amcl_ys, 'b--', label='Estimated Path (AMCL)', linewidth=1.5)
            plt.scatter([0.0], [0.0], color='red', marker='x', s=100, label='Dock / Spawn')
            
            stations = {
                "station_A_001": (-4.0, 2.0),
                "station_A_002": (-4.0, -2.0),
                "station_B_002": (4.0, -2.0),
                "station_B_001": (3.33, 2.13)
            }
            for name, coord in stations.items():
                plt.scatter([coord[0]], [coord[1]], color='purple', marker='o', s=80)
                plt.text(coord[0]+0.1, coord[1]+0.1, name, fontsize=9, fontweight='bold')
                
            plt.title('AMR Trajectory Comparison: AMCL vs Ground Truth')
            plt.xlabel('X (meters)')
            plt.ylabel('Y (meters)')
            plt.grid(True)
            plt.legend()
            
            plot_path = os.path.join(self.output_dir, 'trajectory.png')
            plt.savefig(plot_path)
            plt.close()
            self.get_logger().info(f"Trajectory plot saved to {plot_path}")
            
            # 2. Compute scan statistics
            total_stations = 4
            successful_scans = 0
            scans_summary = []
            
            for row in scan_data:
                ts, station, qr, status = row
                if station == 'dock':
                    continue
                scans_summary.append(f"| {station} | {qr} | {status} | {ts} |")
                if status == 'SUCCESS':
                    successful_scans += 1
                    
            success_rate = (successful_scans / total_stations) * 100.0 if total_stations > 0 else 0.0
            
            # 3. Generate Markdown Report
            report_path = os.path.join(self.output_dir, 'mission_report.md')
            with open(report_path, 'w') as f:
                f.write("# AMR Mission Performance Evaluation Report\n\n")
                f.write(f"**Date/Time (UTC):** {datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                
                f.write("## 1. Executive Summary\n")
                f.write(f"- **Scan Success Rate:** {success_rate:.1f}% ({successful_scans}/{total_stations} stations scanned successfully)\n")
                f.write(f"- **Mean Localization Drift:** {mean_drift*100:.2f} cm\n")
                f.write(f"- **Max Localization Drift:** {max_drift*100:.2f} cm\n")
                f.write(f"- **Final Docking Drift:** {final_drift*100:.2f} cm\n\n")
                
                f.write("## 2. Localization Drift Performance\n")
                f.write("The localization accuracy is evaluated by comparing the AMR's estimated pose from AMCL against the Gazebo Ground Truth sensor telemetry.\n\n")
                f.write("| Metric | Value |\n")
                f.write("| --- | --- |\n")
                f.write(f"| Mean Drift | {mean_drift*100:.2f} cm |\n")
                f.write(f"| Maximum Drift | {max_drift*100:.2f} cm |\n")
                f.write(f"| Final Dock Drift | {final_drift*100:.2f} cm |\n\n")
                
                f.write("## 3. Inventory Station Scan Log\n")
                f.write("| Station ID | Detected QR Content | Status | Timestamp |\n")
                f.write("| --- | --- | --- | --- |\n")
                for summary_line in scans_summary:
                    f.write(summary_line + "\n")
                f.write("\n")
                
                f.write("## 4. Trajectory Visualization\n")
                f.write("Below is the graphical plot displaying the comparison between estimated (AMCL) and ground-truth trajectories:\n\n")
                f.write("![Mission Trajectory](trajectory.png)\n")
                
            self.get_logger().info(f"Mission performance report saved to {report_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate mission report: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MetricsEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Metrics Evaluator interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
