#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import numpy as np

class CoordinateCalibration(Node):
    def __init__(self):
        super().__init__('coordinate_calibration')
        self.get_logger().info("Coordinate Calibration Node Started.")
        self.get_logger().info("Please ensure Gazebo is running and the robot is manually localized in RViz.")

        self.gt_pose = None
        self.amcl_pose = None

        self.gt_sub = self.create_subscription(
            PoseStamped,
            '/ground_truth',
            self.gt_callback,
            10
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

    def gt_callback(self, msg):
        self.gt_pose = msg.pose

    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose
        self.try_calibrate()

    def get_yaw(self, orientation):
        z = orientation.z
        w = orientation.w
        return math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))

    def try_calibrate(self):
        if self.gt_pose is None or self.amcl_pose is None:
            return

        # Gazebo coordinates (Ground Truth)
        x_g = self.gt_pose.position.x
        y_g = self.gt_pose.position.y
        yaw_g = self.get_yaw(self.gt_pose.orientation)

        # Map coordinates (AMCL)
        x_m = self.amcl_pose.position.x
        y_m = self.amcl_pose.position.y
        yaw_m = self.get_yaw(self.amcl_pose.orientation)

        # Calculate rotation difference
        # Delta yaw = yaw_m - yaw_g
        delta_yaw = yaw_m - yaw_g
        # Normalize to [-pi, pi]
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))

        # Calculate translation offset (x0, y0) of Gazebo origin (0,0) in Map frame:
        # x_m = x_g * cos(d_yaw) - y_g * sin(d_yaw) + x0
        # y_m = x_g * sin(d_yaw) + y_g * cos(d_yaw) + y0
        x0 = x_m - (x_g * math.cos(delta_yaw) - y_g * math.sin(delta_yaw))
        y0 = y_m - (x_g * math.sin(delta_yaw) + y_g * math.cos(delta_yaw))

        self.get_logger().info("========================================")
        self.get_logger().info("CALIBRATION SUCCESSFUL!")
        self.get_logger().info(f"Gazebo Pose: x={x_g:.4f}, y={y_g:.4f}, yaw={yaw_g:.4f}")
        self.get_logger().info(f"Map Pose:    x={x_m:.4f}, y={y_m:.4f}, yaw={yaw_m:.4f}")
        self.get_logger().info(f"Offset:      x0={x0:.4f}, y0={y0:.4f}, delta_yaw={delta_yaw:.4f}")
        self.get_logger().info("========================================")

        # Define all Gazebo targets
        gazebo_targets = {
            "dock": (4.369776, 3.527001, -1.87548),
            "station_A_001": (-4.88, 2.0, 0.0),
            "station_A_002": (-4.88, -2.0, 0.0),
            "station_B_001": (4.88, 2.0, 3.14159),
            "station_B_002": (4.88, -2.0, 3.14159),
        }

        print("\n=== MAP COORDINATES FOR WAYPOINTS (Paste into mission_manager.py) ===")
        print("self.waypoints = [")
        for name, (xg, yg, yawg) in gazebo_targets.items():
            if name == "dock":
                continue
            # Transform
            xm = xg * math.cos(delta_yaw) - yg * math.sin(delta_yaw) + x0
            ym = xg * math.sin(delta_yaw) + yg * math.cos(delta_yaw) + y0
            yawm = yawg + delta_yaw
            # Normalize yawm
            yawm = math.atan2(math.sin(yawm), math.cos(yawm))
            print(f'    {{"id": "{name}", "x": {xm:.4f}, "y": {ym:.4f}, "yaw": {yawm:.4f}}},')
        print("]")

        # Transform Dock Pose
        xg, yg, yawg = gazebo_targets["dock"]
        xm = xg * math.cos(delta_yaw) - yg * math.sin(delta_yaw) + x0
        ym = xg * math.sin(delta_yaw) + yg * math.cos(delta_yaw) + y0
        yawm = yawg + delta_yaw
        yawm = math.atan2(math.sin(yawm), math.cos(yawm))
        print(f'\nself.dock_pose = {{"x": {xm:.4f}, "y": {ym:.4f}, "yaw": {yawm:.4f}}}')

        print("\n=== INITIAL AMCL POSE (Paste into navigation.launch.py) ===")
        print(f"x_init = {xm:.6f}")
        print(f"y_init = {ym:.6f}")
        print(f"yaw_init = {yawm:.6f}")
        print("========================================\n")

        # Shutdown node after printing
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateCalibration()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
