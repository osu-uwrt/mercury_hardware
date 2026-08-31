#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Point, Vector3, PoseWithCovarianceStamped

RATE_HZ = 30
VARIANCE = 1.66e-03
TOPIC_NAME = "tag_odom/pose"


def v3ToPoint(v3: Vector3):
    pt = Point()
    pt.x = v3.x
    pt.y = v3.y
    pt.z = v3.z
    return pt


class TagOdomPublisher(Node):
    def __init__(self):
        super().__init__("tag_odom_publisher")

        # what robot?
        # robot name is namespace without leading /
        self.robotName = self.get_namespace()[1:]
        self.robotName = self.robotName if not self.robotName.endswith(
            '/') else self.robotName[0: len(self.robotName) - 1]  # remove trailing /

        # timer
        self.create_timer(1 / RATE_HZ, self.timerCb)

        # publisher
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, TOPIC_NAME, qos_profile_sensor_data)

        # tf
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.get_logger().info(
            f"tag_odom started with robot name \"{self.robotName}\"")

    def timerCb(self):
        fromFrame = f"{self.robotName}/base_link"
        toFrame = "estimated_origin_frame"
        try:
            # look up transform
            transform = self.buffer.lookup_transform(
                toFrame, fromFrame, Time())

            # pack msg
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.pose.pose.position = v3ToPoint(transform.transform.translation)
            msg.pose.pose.orientation = transform.transform.rotation
            msg.pose.covariance[0] = VARIANCE
            msg.pose.covariance[7] = VARIANCE
            msg.pose.covariance[14] = VARIANCE
            msg.pose.covariance[21] = VARIANCE
            msg.pose.covariance[28] = VARIANCE
            msg.pose.covariance[35] = VARIANCE
            self.pub.publish(msg)
        except TransformException as ex:
            self.get_logger().error(
                f"Could not look up transform from {fromFrame} to {toFrame}: {ex}", skip_first=True, throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)
    node = TagOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
