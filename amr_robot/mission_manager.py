#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
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
        self.current_pose_x = None
        self.current_pose_y = None
        self.current_pose_yaw = None
        self.latest_detected_qr = None
        self.qr_received_time = 0.0
        self.latest_raw_qr = None

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
        self.qr_raw_sub = self.node.create_subscription(
            String,
            '/detected_qr_raw',
            self.qr_raw_callback,
            10
        )

        # Publishers
        self.log_pub = self.node.create_publisher(String, '/log_scan', 10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # -----------------------------------------------------------------------
        # HARDCODED MAP-FRAME WAYPOINTS (ground-truth verified via RViz2 selector)
        # -----------------------------------------------------------------------
        # How these were derived:
        #   - User identified Station A_001 center in RViz2 point cloud: (9.66, 10.197) in /map
        #   - Station A_001 is at Gazebo (-4.88, 2.0)  =>  offset: dx=14.54, dy=8.197
        #   - Station B_001 is at Gazebo ( 4.88, 2.0)  =>  map: (19.42, 10.197) [wall edge]
        #   - Approach = 1.5 m in front of each station, robot faces the QR code.
        #
        #  Station A QR faces +x in world  => robot approaches from +x, faces -x (yaw=pi)
        #  Station B QR faces -x in world  => robot approaches from -x, faces +x (yaw=0)
        # -----------------------------------------------------------------------

        # Approach waypoints – robot navigates HERE, then scans QR
        # -----------------------------------------------------------------------
        # CALIBRATED via RViz2 Publish Point (user ground-truth measurements):
        #
        # Station A_001:
        #   QR wall center (point cloud selector): map (9.66, 10.197)
        #   Ideal approach (publish point):        map (9.2,  11.3)
        #   => Robot stands ~1.2m from QR, yaw = -67.4 deg (-1.176 rad)
        #
        # Station B_001:
        #   Ideal approach (publish point):        map (5.2, 17.6)
        #   yaw = yaw_A + pi = 112.6 deg (1.966 rad)  [B rotated 180 from A]
        # -----------------------------------------------------------------------
        self.waypoints = [
            {
                "id": "station_A_001",
                "x": 9.5,      # ground-truth from RViz2 publish point
                "y": 11.3,
                "yaw": -1.176  # atan2(10.197-11.3, 9.66-9.2) = faces QR wall
            },
            {
                "id": "station_B_001",
                "x": 5.2,     # ground-truth from RViz2 publish point
                "y": 17.6,
                "yaw": 2.069  # corrected by user: 118.5 deg (was 112.6, +5.9 deg tweak)
            },
        ]

        # Dock = robot spawn in /map frame
        self.dock_pose = {"x": 18.91, "y": 11.724, "yaw": -1.87548}

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

    def qr_raw_callback(self, msg):
        try:
            self.latest_raw_qr = json.loads(msg.data)
        except Exception as e:
            self.node.get_logger().error(f"Failed to parse raw QR: {e}")

    def align_to_qr(self, target_station_id):
        self.node.get_logger().info(f"Starting alignment to QR code for {target_station_id}...")
        start_time = time.time()
        timeout = 10.0
        consecutive_aligned = 0
        required_consecutive = 5
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
            # Read latest raw QR status
            detected = False
            offset = 0.0
            info = ""
            
            if self.latest_raw_qr:
                detected = self.latest_raw_qr.get("detected", False)
                info = self.latest_raw_qr.get("info", "")
                offset = self.latest_raw_qr.get("offset", 0.0)
                
            twist = Twist()
            
            if detected and info == target_station_id:
                # Proportional control
                # angular_vel = -0.5 * offset. Limit max to 0.1 rad/s.
                kp = 0.5
                angular_z = -kp * offset
                angular_z = max(-0.1, min(0.1, angular_z))
                twist.angular.z = angular_z
                self.node.get_logger().info(f"Aligning: target={info}, offset={offset:.3f}, cmd_vel={angular_z:.3f}")
                
                if abs(offset) < 0.05:
                    consecutive_aligned += 1
                else:
                    consecutive_aligned = 0
            else:
                # Not detected or wrong station. Rotate slowly to search (e.g. 0.05 rad/s)
                twist.angular.z = 0.05
                consecutive_aligned = 0
                self.node.get_logger().info("QR not visible, searching...")
                
            self.cmd_vel_pub.publish(twist)
            
            if consecutive_aligned >= required_consecutive:
                self.node.get_logger().info(f"Successfully aligned to {target_station_id}!")
                stop_twist = Twist()
                self.cmd_vel_pub.publish(stop_twist)
                return True
                
            time.sleep(0.1)
            
        self.node.get_logger().warn(f"Alignment timeout for {target_station_id}!")
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        return False

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

        # 2. Log hardcoded waypoints for confirmation
        self.node.get_logger().info("=== Mission waypoints (hardcoded map-frame) ===")
        for wp in self.waypoints:
            self.node.get_logger().info(
                f"  {wp['id']}: Map({wp['x']:.3f}, {wp['y']:.3f}, "
                f"{math.degrees(wp['yaw']):.1f}deg)"
            )
        self.node.get_logger().info(
            f"  Dock: Map({self.dock_pose['x']:.3f}, {self.dock_pose['y']:.3f}, "
            f"{math.degrees(self.dock_pose['yaw']):.1f}deg)"
        )

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

            self.node.get_logger().info(f"Arrived at {station_id}. Initiating alignment phase...")
            
            # Align robot with QR first
            self.align_to_qr(station_id)
            
            # Scan attempt
            success = self.perform_scan_attempt(station_id)
            
            if not success:
                # Retry/Nudge logic
                self.node.get_logger().info(f"Scan failed/mismatched for {station_id}. Initiating nudge retry...")
                
                # Calculate nudge pose (0.15m closer to the target station)
                nudge_dist = 0.15
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
                    self.node.get_logger().info("Nudge completed. Re-aligning and initiating scan retry...")
                    self.align_to_qr(station_id)
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
            self.log_scan("dock", "COMPLETE")
        else:
            self.node.get_logger().warn(f"Failed to return to dock. Result: {final_result}")
            self.log_scan("dock", "FAILED")

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
