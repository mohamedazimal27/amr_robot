#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import sys

def quat_to_yaw(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

class TransformTest(Node):
    def __init__(self):
        super().__init__('transform_test')
        self.gz_pose = None
        self.amcl_pose = None
        
        self.create_subscription(PoseStamped, '/ground_truth', self.gz_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)

    def gz_cb(self, msg):
        self.gz_pose = msg.pose

    def amcl_cb(self, msg):
        self.amcl_pose = msg.pose.pose

def main():
    rclpy.init()
    node = TransformTest()
    
    print("Waiting for /ground_truth and /amcl_pose...")
    while rclpy.ok() and (node.gz_pose is None or node.amcl_pose is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        
    gz = node.gz_pose
    amcl = node.amcl_pose
    
    xg, yg = gz.position.x, gz.position.y
    yaw_g = quat_to_yaw(gz.orientation.z, gz.orientation.w)
    
    xm, ym = amcl.position.x, amcl.position.y
    yaw_m = quat_to_yaw(amcl.orientation.z, amcl.orientation.w)
    
    d = yaw_m - yaw_g
    d = math.atan2(math.sin(d), math.cos(d))
    
    x0 = xm - (xg * math.cos(d) - yg * math.sin(d))
    y0 = ym - (xg * math.sin(d) + yg * math.cos(d))
    
    print("\n--- Current Poses ---")
    print(f"Gazebo Ground Truth: x={xg:.4f}, y={yg:.4f}, yaw={math.degrees(yaw_g):.1f}°")
    print(f"AMCL Map Pose:       x={xm:.4f}, y={ym:.4f}, yaw={math.degrees(yaw_m):.1f}°")
    print("\n--- Computed Dynamic Transform ---")
    print(f"delta_yaw: {math.degrees(d):.4f}°")
    print(f"translation offset: x0={x0:.4f}, y0={y0:.4f}")
    
    # Let's map the station coords
    # Station A QR panel is on the LEFT wall (x ≈ -4.88, y = 2.0), robot faces +x (approach at x = -3.5, y = 2.0, yaw = pi)
    # Station B QR panel is on the RIGHT wall (x ≈ 4.88, y = 2.0), robot faces -x (approach at x = 3.5, y = 2.0, yaw = 0.0)
    stations = [
        {"id": "station_A_001_approach", "gz_x": -3.5, "gz_y": 2.0, "gz_yaw": math.pi},
        {"id": "station_B_001_approach", "gz_x":  3.5, "gz_y": 2.0, "gz_yaw": 0.0},
    ]
    
    print("\n--- Computed Station Map Goals ---")
    for s in stations:
        mx = s["gz_x"] * math.cos(d) - s["gz_y"] * math.sin(d) + x0
        my = s["gz_x"] * math.sin(d) + s["gz_y"] * math.cos(d) + y0
        myaw = math.atan2(math.sin(s["gz_yaw"] + d), math.cos(s["gz_yaw"] + d))
        qz = math.sin(myaw / 2.0)
        qw = math.cos(myaw / 2.0)
        
        print(f"\n{s['id']}:")
        print(f"  Position: x={mx:.4f}, y={my:.4f}, yaw={math.degrees(myaw):.1f}°")
        print(f"  Quaternion: z={qz:.6f}, w={qw:.6f}")
        print(f"  Command: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{{pose: {{header: {{frame_id: 'map'}}, pose: {{position: {{x: {mx:.4f}, y: {my:.4f}, z: 0.0}}, orientation: {{z: {qz:.6f}, w: {qw:.6f}}}}}}}}}\"")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
