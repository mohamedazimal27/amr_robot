#!/usr/bin/env python3
"""
compute_and_send_station_goal.py

Run this AFTER:
  1. ros2 launch amr_robot gazebo.launch.py world:=opil_factory
  2. ros2 launch amr_robot navigation.launch.py map:=opil_factory
  3. Set 2D Pose Estimate in RViz so laser scans align with the map walls

Usage:
  # Compute and PRINT station goals (no navigation):
  python3 scratch/compute_and_send_station_goal.py

  # Send goal to station_A_001:
  python3 scratch/compute_and_send_station_goal.py A

  # Send goal to station_B_001:
  python3 scratch/compute_and_send_station_goal.py B
"""

import sys, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# ─── Robot spawn in Gazebo world frame ────────────────────────────────────────
GZ_SPAWN = (4.369776, 3.527001, -1.87548)   # x, y, yaw

# ─── Station approach points in Gazebo world frame ────────────────────────────
# "Approach" = stop ~1 m in front of the QR panel, facing it.
# Station A QR panel is on the LEFT wall  (x ≈ -4.88), robot faces +x → yaw=π
# Station B QR panel is on the RIGHT wall (x ≈  4.88), robot faces -x → yaw=0
GZ_GOALS = {
    'A': {"id": "station_A_001", "gz_x": -3.5, "gz_y": 2.0, "gz_yaw": math.pi},
    'B': {"id": "station_B_001", "gz_x":  3.5, "gz_y": 2.0, "gz_yaw": 0.0},
}


def quat_to_yaw(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)   # (z, w)


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.amcl_pose = None
        self._sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._cb, 10)
        self._pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def _cb(self, msg):
        self.amcl_pose = msg

    def wait_for_pose(self, timeout=30.0):
        self.get_logger().info("Waiting for /amcl_pose ...")
        t = time.time()
        while rclpy.ok() and self.amcl_pose is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t > timeout:
                self.get_logger().error("Timeout! Is Nav2 running and robot localized?")
                return False
        return True

    def compute_transform(self):
        """Compute rigid transform: Gazebo world → /map frame."""
        p = self.amcl_pose.pose.pose
        xm = p.position.x
        ym = p.position.y
        yaw_m = quat_to_yaw(p.orientation.z, p.orientation.w)

        xg, yg, yaw_g = GZ_SPAWN
        d = math.atan2(math.sin(yaw_m - yaw_g), math.cos(yaw_m - yaw_g))
        x0 = xm - (xg * math.cos(d) - yg * math.sin(d))
        y0 = ym - (xg * math.sin(d) + yg * math.cos(d))

        self.get_logger().info(
            f"AMCL pose: ({xm:.4f}, {ym:.4f}, {math.degrees(yaw_m):.1f}°)")
        self.get_logger().info(
            f"GZ spawn:  ({xg:.4f}, {yg:.4f}, {math.degrees(yaw_g):.1f}°)")
        self.get_logger().info(
            f"Transform: delta_yaw={math.degrees(d):.1f}°, offset=({x0:.4f}, {y0:.4f})")
        return d, x0, y0

    def gz_to_map(self, gz_x, gz_y, gz_yaw, d, x0, y0):
        mx = gz_x * math.cos(d) - gz_y * math.sin(d) + x0
        my = gz_x * math.sin(d) + gz_y * math.cos(d) + y0
        myaw = math.atan2(math.sin(gz_yaw + d), math.cos(gz_yaw + d))
        return mx, my, myaw

    def print_all_goals(self, d, x0, y0):
        print()
        print("=" * 60)
        print("COMPUTED MAP-FRAME GOALS")
        print("=" * 60)
        for key, s in GZ_GOALS.items():
            mx, my, myaw = self.gz_to_map(s['gz_x'], s['gz_y'], s['gz_yaw'], d, x0, y0)
            qz, qw = yaw_to_quat(myaw)
            print(f"\n[Station {key}] {s['id']}")
            print(f"  Map pose: x={mx:.4f}  y={my:.4f}  yaw={math.degrees(myaw):.1f}°")
            print(f"  Quaternion: z={qz:.6f}  w={qw:.6f}")
            print()
            print("  ── Send manually with: ──────────────────────────────")
            print(f"  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\")
            print(f"    \"{{pose: {{header: {{frame_id: 'map'}}, pose: {{position: {{x: {mx:.4f}, y: {my:.4f}, z: 0.0}}, orientation: {{z: {qz:.6f}, w: {qw:.6f}}}}}}}}}\"")
        print("=" * 60)
        print()

    def send_goal(self, key, d, x0, y0):
        s = GZ_GOALS[key]
        mx, my, myaw = self.gz_to_map(s['gz_x'], s['gz_y'], s['gz_yaw'], d, x0, y0)
        qz, qw = yaw_to_quat(myaw)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = mx
        goal.pose.position.y = my
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0

        self.get_logger().info(
            f"Sending goal to {s['id']}: Map({mx:.4f}, {my:.4f}, {math.degrees(myaw):.1f}°)")
        for _ in range(3):           # publish a few times to ensure RViz receives it
            self._pub.publish(goal)
            time.sleep(0.1)
        self.get_logger().info("Goal published to /goal_pose. Check RViz for the navigation goal marker.")


def main():
    rclpy.init()
    node = GoalSender()

    if not node.wait_for_pose():
        rclpy.shutdown()
        sys.exit(1)

    d, x0, y0 = node.compute_transform()
    node.print_all_goals(d, x0, y0)

    # If a station key was passed as argument, also send the goal
    if len(sys.argv) > 1:
        key = sys.argv[1].upper()
        if key in GZ_GOALS:
            node.send_goal(key, d, x0, y0)
        else:
            print(f"Unknown station key '{key}'. Use A or B.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
