import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.get_logger().info("Image saver started, waiting for frame...")

    def callback(self, msg):
        self.get_logger().info("Frame received!")
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite('/home/mohamed-azimal/ros2_ws/src/amr_robot/scratch/camera_frame.png', cv_img)
            self.get_logger().info("Saved frame to scratch/camera_frame.png")
            sys.exit(0)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            sys.exit(1)

def main():
    rclpy.init()
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
