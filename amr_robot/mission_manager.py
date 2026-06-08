#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import math
import json
import time
from datetime import datetime

def create_pose_stamped(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    
    # 2D yaw to quaternion
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    return pose

class MissionManager:
    def __init__(self, navigator):
        self.navigator = navigator
        self.node = navigator # BasicNavigator is a Node
        
        # State variables
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0
        self.latest_detected_qr = None
        self.qr_received_time = 0.0

        # Subscriptions on navigator node
        self.pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.qr_sub = self.node.create_subscription(
            String,
            '/detected_qr',
            self.qr_callback,
            10
        )

        # Publisher for scan events
        self.log_pub = self.node.create_publisher(String, '/log_scan', 10)

        # Mission parameters (Default coordinates for opil_factory)
        self.waypoints = [
            {"id": "station_A_001", "x": -4.0, "y": 2.0, "yaw": math.pi},
            {"id": "station_A_002", "x": -4.0, "y": -2.0, "yaw": math.pi},
            {"id": "station_B_002", "x": 4.0, "y": -2.0, "yaw": 0.0},
            {"id": "station_B_001", "x": 4.0, "y": 2.0, "yaw": 0.0}
        ]
        self.dock_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose_x = pose.position.x
        self.current_pose_y = pose.position.y
        z = pose.orientation.z
        w = pose.orientation.w
        self.current_pose_yaw = math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))

    def qr_callback(self, msg):
        self.latest_detected_qr = msg.data
        self.qr_received_time = time.time()
        self.node.get_logger().info(f"Detected QR Code: {self.latest_detected_qr}")

    def log_scan(self, station_id, status):
        log_data = {
            'timestamp': datetime.utcnow().isoformat(),
            'station_id': station_id,
            'qr_content': self.latest_detected_qr if self.latest_detected_qr else '',
            'robot_pose_x': self.current_pose_x,
            'robot_pose_y': self.current_pose_y,
            'robot_pose_yaw': self.current_pose_yaw,
            'status': status
        }
        msg = String()
        msg.data = json.dumps(log_data)
        self.log_pub.publish(msg)
        self.node.get_logger().info(f"Published scan log event: Station={station_id}, Status={status}")

    def execute_mission(self):
        # 1. Wait for Nav2
        self.node.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.node.get_logger().info("Nav2 is active!")

        # 2. Set Initial Pose (from spawn location in opil_factory)
        initial_pose = create_pose_stamped(self.navigator, 4.0, 4.0, 0.0)
        self.navigator.setInitialPose(initial_pose)
        self.node.get_logger().info("Set initial pose to (4.0, 4.0, 0.0)")

        # Allow time for localization to initialize
        time.sleep(2.0)
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # 3. Traverse Waypoints
        for wp in self.waypoints:
            station_id = wp["id"]
            self.node.get_logger().info(f"=== Starting navigation to {station_id} ===")
            
            # Navigate to waypoint
            goal = create_pose_stamped(self.navigator, wp["x"], wp["y"], wp["yaw"])
            self.navigator.goToPose(goal)

            # Monitor progress
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                feedback = self.navigator.getFeedback()
                if feedback:
                    # Optional: print navigation feedback
                    pass

            result = self.navigator.getResult()
            if result != TaskResult.SUCCEEDED:
                self.node.get_logger().warn(f"Failed to navigate to {station_id}! Result: {result}")
                self.log_scan(station_id, "TIMEOUT")
                continue

            self.node.get_logger().info(f"Arrived at {station_id}. Initiating scan phase...")
            
            # Scan attempt
            success = self.perform_scan_attempt(station_id)
            
            if not success:
                # Retry/Nudge logic
                self.node.get_logger().info(f"Scan failed/mismatched for {station_id}. Initiating nudge retry...")
                
                # Calculate nudge pose (0.1m closer to the target station)
                nudge_dist = 0.15 # 15cm closer
                nudge_x = wp["x"] + nudge_dist * math.cos(wp["yaw"])
                nudge_y = wp["y"] + nudge_dist * math.sin(wp["yaw"])
                
                self.node.get_logger().info(f"Nudging to pose: x={nudge_x:.2f}, y={nudge_y:.2f}")
                nudge_goal = create_pose_stamped(self.navigator, nudge_x, nudge_y, wp["yaw"])
                self.navigator.goToPose(nudge_goal)
                
                # Monitor nudge navigation
                while not self.navigator.isTaskComplete():
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                
                nudge_result = self.navigator.getResult()
                if nudge_result == TaskResult.SUCCEEDED:
                    self.node.get_logger().info("Nudge completed. Initiating scan retry...")
                    self.perform_scan_attempt(station_id)
                else:
                    self.node.get_logger().warn("Nudge navigation failed!")
                    self.log_scan(station_id, "TIMEOUT")

        # 4. Return to Dock
        self.node.get_logger().info("=== All stations visited. Returning to dock ===")
        dock_goal = create_pose_stamped(self.navigator, self.dock_pose["x"], self.dock_pose["y"], self.dock_pose["yaw"])
        self.navigator.goToPose(dock_goal)
        
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        final_result = self.navigator.getResult()
        if final_result == TaskResult.SUCCEEDED:
            self.node.get_logger().info("Successfully returned to dock! Mission Completed.")
        else:
            self.node.get_logger().warn(f"Failed to return to dock. Result: {final_result}")

    def perform_scan_attempt(self, station_id):
        # Clear previous detection
        self.latest_detected_qr = None
        
        # Halt and wait for 3.0 seconds to detect
        start_time = time.time()
        scan_duration = 3.0
        
        self.node.get_logger().info(f"Waiting {scan_duration} seconds for QR code detection...")
        while time.time() - start_time < scan_duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_detected_qr is not None:
                # Received a code!
                if self.latest_detected_qr == station_id:
                    self.node.get_logger().info(f"MATCH SUCCESS: Detected expected QR code: {self.latest_detected_qr}")
                    self.log_scan(station_id, "SUCCESS")
                    return True
                else:
                    self.node.get_logger().warn(f"MISMATCH: Expected {station_id}, but detected {self.latest_detected_qr}")
                    # Keep looking in case we detect the right one or decide mismatch immediately
            time.sleep(0.05)
            
        # If we got here and didn't return True, it's either a timeout or a mismatch
        if self.latest_detected_qr is not None:
            self.log_scan(station_id, "MISMATCH")
            return False
        else:
            self.node.get_logger().warn("Scan TIMEOUT: No QR code detected.")
            self.log_scan(station_id, "TIMEOUT")
            return False

def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    manager = MissionManager(navigator)
    
    try:
        manager.execute_mission()
    except KeyboardInterrupt:
        manager.node.get_logger().info("Mission interrupted by user.")
    finally:
        navigator.lifecycleShutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
