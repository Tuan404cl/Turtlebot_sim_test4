#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TF Relay & Controller Watchdog Node cho mô phỏng TurtleBot 4 trong Gazebo
Chức năng:
1. Lắng nghe topic `/odom` (nav_msgs/msg/Odometry) và phát trực tiếp biến đổi `odom -> base_link`
   lên `/tf` để đảm bảo cây TF luôn liên thông 100% cho Nav2 (`local_costmap`, `controller_server`)
   và `amcl` ngay từ khi khởi động.
2. Cung cấp các dummy service phần cứng (`start_motor`, `oakd/start_camera`...) phục vụ simulator.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, Trigger
import tf2_ros


class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        self.declare_parameter('robot_name', 'turtlebot4')
        self.robot_name = self.get_parameter('robot_name').value

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.last_odom_msg = None

        # Lắng nghe /odom với cả BestEffort và Reliable
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_cb, odom_qos
        )

        # Timer 25 Hz đảm bảo luôn có transform odom -> base_link ngay cả khi robot đang đứng yên trên dock
        self.tf_timer = self.create_timer(0.04, self._publish_fallback_odom_tf)

        # Mock dummy services để khắc phục triệt để lỗi phần cứng không có trong Gazebo
        self.srv_start_motor = self.create_service(Empty, 'start_motor', self._empty_cb)
        self.srv_stop_motor = self.create_service(Empty, 'stop_motor', self._empty_cb)
        self.srv_start_cam = self.create_service(Trigger, 'oakd/start_camera', self._trigger_cb)
        self.srv_stop_cam = self.create_service(Trigger, 'oakd/stop_camera', self._trigger_cb)

        self.get_logger().info('TF Relay sẵn sàng: Đảm bảo kết nối odom -> base_link & Mock Hardware Services.')

    def _odom_cb(self, msg: Odometry):
        self.last_odom_msg = msg
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _publish_fallback_odom_tf(self):
        """Nếu diffdrive_controller đang khởi tạo hoặc chưa có /odom mới, giữ cây TF odom -> base_link liên thông."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        if self.last_odom_msg is not None:
            t.transform.translation.x = self.last_odom_msg.pose.pose.position.x
            t.transform.translation.y = self.last_odom_msg.pose.pose.position.y
            t.transform.translation.z = self.last_odom_msg.pose.pose.position.z
            t.transform.rotation = self.last_odom_msg.pose.pose.orientation
        else:
            t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _empty_cb(self, request, response):
        return response

    def _trigger_cb(self, request, response):
        response.success = True
        response.message = 'Simulation dummy service OK'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TFRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
