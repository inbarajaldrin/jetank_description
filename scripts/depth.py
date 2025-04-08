#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

from PIL import Image as PILImage
import numpy as np
import cv2

from transformers import pipeline

class DepthAnythingPipelineNode(Node):
    def __init__(self):
        super().__init__('depth_anything_pipeline_node')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/depth/image_raw',
            10
        )

        self.get_logger().info("Loading Depth-Anything-V2-Small model from HuggingFace...")
        self.pipe = pipeline(
            task="depth-estimation",
            model="depth-anything/Depth-Anything-V2-Small-hf"
        )
        self.get_logger().info("Model loaded successfully!")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

            # Run Depth-Anything pipeline
            depth_pil = self.pipe(pil_image)["depth"]

            # Convert to numpy array
            depth_np = np.array(depth_pil)

            # Normalize to 8-bit grayscale
            depth_norm = cv2.normalize(depth_np, None, 0, 255, cv2.NORM_MINMAX)
            depth_uint8 = depth_norm.astype(np.uint8)

            # Publish as ROS Image
            depth_msg = self.bridge.cv2_to_imgmsg(depth_uint8, encoding='mono8')
            depth_msg.header = msg.header
            self.depth_pub.publish(depth_msg)

            self.get_logger().info("Published depth image.")
        except Exception as e:
            self.get_logger().error(f"Error during depth estimation: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingPipelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
