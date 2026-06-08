#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sqlite3
import json
import os
from datetime import datetime

class InventoryLogger(Node):
    def __init__(self):
        super().__init__('inventory_logger')
        
        # Declare parameters
        self.declare_parameter('db_path', '/home/mohamed-azimal/ros2_ws/src/amr_robot/maps/inventory.db')
        self.db_path = os.path.expanduser(self.get_parameter('db_path').value)
        
        self.get_logger().info(f"Using database path: {self.db_path}")
        
        # Ensure the directories exist
        db_dir = os.path.dirname(self.db_path)
        if db_dir and not os.path.exists(db_dir):
            os.makedirs(db_dir, exist_ok=True)
            
        # Initialize SQLite database
        self.init_db()
        
        # Subscription to consolidated log scan events
        self.scan_sub = self.create_subscription(
            String,
            '/log_scan',
            self.log_scan_callback,
            10
        )
        
        self.get_logger().info("Inventory Logger Node initialized and listening on /log_scan.")

    def init_db(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
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
            conn.commit()
            conn.close()
            self.get_logger().info("SQLite Database initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize SQLite Database: {str(e)}")

    def log_scan_callback(self, msg):
        try:
            # Parse JSON message
            data = json.loads(msg.data)
            
            # Extract fields
            timestamp = data.get('timestamp', datetime.utcnow().isoformat())
            station_id = data.get('station_id', 'UNKNOWN')
            qr_content = data.get('qr_content', '')
            pose_x = data.get('robot_pose_x', 0.0)
            pose_y = data.get('robot_pose_y', 0.0)
            pose_yaw = data.get('robot_pose_yaw', 0.0)
            status = data.get('status', 'UNKNOWN')
            
            # Log to SQLite
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute('''
                INSERT INTO scans (timestamp, station_id, qr_content, robot_pose_x, robot_pose_y, robot_pose_yaw, status)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', (timestamp, station_id, qr_content, pose_x, pose_y, pose_yaw, status))
            conn.commit()
            conn.close()
            
            self.get_logger().info(
                f"Successfully logged scan: Station={station_id}, QR={qr_content}, "
                f"Pose=({pose_x:.2f}, {pose_y:.2f}, {pose_yaw:.2f}), Status={status}"
            )
        except Exception as e:
            self.get_logger().error(f"Error logging scan: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = InventoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
