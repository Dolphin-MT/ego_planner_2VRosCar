#!/usr/bin/env python3

import rclpy
import rclpy.time

from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


class TfToOdomWorld(Node):
    def __init__(self):
        super().__init__('tf_to_odom_world')

        self.map_frame = 'map'
        self.base_frame = 'base_footprint'

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # ego_replan_fsm.cpp 默认订阅 odom_world
        self.pub = self.create_publisher(Odometry, '/odom_world', 10)

        self.timer = self.create_timer(0.05, self.on_timer)

    def on_timer(self):
        try:
            tf_msg = self.buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time())
        except Exception:
            # map -> base_footprint 还没有准备好
            return

        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame

        # 位置
        msg.pose.pose.position.x = tf_msg.transform.translation.x
        msg.pose.pose.position.y = tf_msg.transform.translation.y

        # 二维小车：强制 z = 0
        msg.pose.pose.position.z = 0.0

        # 姿态
        msg.pose.pose.orientation = tf_msg.transform.rotation

        # 速度先全部置零
        # 后面如果需要更准确的初速度，再从 /odom_combined 转换
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TfToOdomWorld()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()