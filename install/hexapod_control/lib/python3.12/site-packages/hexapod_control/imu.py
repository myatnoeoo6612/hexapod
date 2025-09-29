#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


class DynamicPoseToImu(Node):
    def __init__(self):
        super().__init__('imu')

        # Subscriber to bridged pose array
        self.subscription = self.create_subscription(
            PoseArray,
            '/world/default/dynamic_pose/info',
            self.pose_callback,
            10
        )

        # Publisher for IMU
        self.publisher_imu = self.create_publisher(PoseStamped, '/imu_pose', 100)


        self.get_logger().info("Node started: listening to /world/default/dynamic_pose/info")

    def pose_callback(self, msg: PoseArray):
        # Make sure at least 2 poses exist
        if len(msg.poses) < 2:
            self.get_logger().warn("Not enough poses in message")
            return

        pose = msg.poses[1]  # second pose

        # Publish only position + orientation as PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "imu"

        pose_msg.pose.position = pose.position
        pose_msg.pose.orientation = pose.orientation

        self.publisher_imu.publish(pose_msg)



def main(args=None):
    rclpy.init(args=args)
    node = DynamicPoseToImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
