#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import qos_profile_sensor_data

class SyncDepthRepublisher(Node):
    def __init__(self):
        super().__init__('depth_sync_republisher')

        self.rgb_sub = Subscriber(self, Image, '/camera')
        self.depth_sub = Subscriber(self, Image, '/depth/image_raw')
        self.info_sub = Subscriber(self, CameraInfo, '/camera_info')

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=30,
            slop=0.5
        )
        self.ts.registerCallback(self.synced_callback)

        self.rgb_pub = self.create_publisher(Image, '/synced/camera', 10)
        self.depth_pub = self.create_publisher(Image, '/synced/depth', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/synced/camera_info', 10)

        self.get_logger().info("Syncing and republishing topics with frame_id = 'CAMERA_OPTICAL'...")

    def synced_callback(self, rgb, depth, info):
        # Overwrite frame_id to match optical convention
        rgb.header.frame_id = "CAMERA_OPTICAL"
        depth.header.frame_id = "CAMERA_OPTICAL"
        info.header.frame_id = "CAMERA_OPTICAL"

        self.rgb_pub.publish(rgb)
        self.depth_pub.publish(depth)
        self.info_pub.publish(info)

def main(args=None):
    rclpy.init(args=args)
    node = SyncDepthRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
