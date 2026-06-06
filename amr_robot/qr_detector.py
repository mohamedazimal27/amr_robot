#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

class QrDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        
        # Parameters
        self.declare_parameter('processing_rate', 10.0) # Hz
        self.declare_parameter('velocity_threshold', 0.05) # m/s
        self.declare_parameter('center_gate_tolerance', 0.20) # 20%
        
        self.rate = self.get_parameter('processing_rate').value
        self.vel_threshold = self.get_parameter('velocity_threshold').value
        self.gate_tolerance = self.get_parameter('center_gate_tolerance').value

        # CV Bridge
        self.bridge = CvBridge()
        
        # QR Code Detector
        self.detector = cv2.QRCodeDetector()

        # State variables
        self.latest_image = None
        self.current_linear_vel = 0.0

        # Subscriptions
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.qr_pub = self.create_publisher(String, '/detected_qr', 10)
        self.annotated_img_pub = self.create_publisher(Image, '/qr_detection_image', 10)

        # Timer for throttled processing
        self.timer = self.create_timer(1.0 / self.rate, self.process_image)
        
        self.get_logger().info('QR Detector node initialized.')

    def image_callback(self, msg):
        self.latest_image = msg

    def odom_callback(self, msg):
        # Calculate linear velocity magnitude (X and Y components)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_linear_vel = np.sqrt(vx**2 + vy**2)

    def process_image(self):
        if self.latest_image is None:
            return

        try:
            # Convert ROS Image message to OpenCV image
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return

        height, width, _ = cv_img.shape
        cx_img, cy_img = width / 2.0, height / 2.0
        
        # Center gate bounding box limits
        dx = width * self.gate_tolerance
        dy = height * self.gate_tolerance
        x_min, x_max = cx_img - dx, cx_img + dx
        y_min, y_max = cy_img - dy, cy_img + dy

        # Draw center gate rectangle (light blue/cyan) for visual aid
        cv2.rectangle(
            cv_img, 
            (int(x_min), int(y_min)), 
            (int(x_max), int(y_max)), 
            (255, 255, 0), 
            1
        )

        # Decode QR codes using OpenCV's native QRCodeDetector
        retval, decoded_info, points, _ = self.detector.detectAndDecodeMulti(cv_img)

        # Draw current velocity info on the image
        vel_text = f"Vel: {self.current_linear_vel:.3f} m/s (Max: {self.vel_threshold})"
        color = (0, 255, 0) if self.current_linear_vel < self.vel_threshold else (0, 0, 255)
        cv2.putText(cv_img, vel_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if retval:
            for i in range(len(decoded_info)):
                info = decoded_info[i]
                if not info:
                    continue
                
                pts = points[i].astype(np.int32)

                # Draw polygon around barcode
                cv2.polylines(cv_img, [pts], True, (0, 0, 255), 2)

                # Compute centroid of barcode
                centroid_x = np.mean(pts[:, 0])
                centroid_y = np.mean(pts[:, 1])

                # Draw centroid marker
                cv2.circle(cv_img, (int(centroid_x), int(centroid_y)), 5, (0, 0, 255), -1)

                # Gate checks
                vel_gate = self.current_linear_vel < self.vel_threshold
                center_gate = (x_min <= centroid_x <= x_max) and (y_min <= centroid_y <= y_max)

                # Build status text
                status_parts = []
                if vel_gate:
                    status_parts.append("VEL_OK")
                else:
                    status_parts.append("VEL_FAIL")
                if center_gate:
                    status_parts.append("CENTER_OK")
                else:
                    status_parts.append("CENTER_FAIL")
                status_str = f"[{', '.join(status_parts)}]"

                if vel_gate and center_gate:
                    # Target detected & verified! Highlight green
                    cv2.polylines(cv_img, [pts], True, (0, 255, 0), 3)
                    cv2.putText(
                        cv_img, 
                        f"{info} - ACTIVE", 
                        (pts[0][0], pts[0][1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        (0, 255, 0), 
                        2
                    )
                    
                    # Publish the QR code content
                    qr_msg = String()
                    qr_msg.data = info
                    self.qr_pub.publish(qr_msg)
                else:
                    # Highlight red (found but gated)
                    cv2.putText(
                        cv_img, 
                        f"{info} - GATED {status_str}", 
                        (pts[0][0], pts[0][1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        (0, 0, 255), 
                        1
                    )

        # Publish annotated image for visualization
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            annotated_msg.header = self.latest_image.header
            self.annotated_img_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = QrDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
