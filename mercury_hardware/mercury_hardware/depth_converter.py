#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from riptide_msgs2.msg import Depth
import tf2_ros
import transforms3d as tf3d
import numpy as np


class depthConverter(Node):
    def __init__(self):
        super().__init__('depth_converter')
        self.sub = self.create_subscription(
            Depth, "state/depth/raw", self.depthCb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "depth/pose", qos_profile_sensor_data)
        self.namespace = self.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.b2pVector = None

    def depthCb(self, msg):
        try:
            # Rotation from base frame to odom
            b2oOrientation = self.tfBuffer.lookup_transform(
                'odom', self.namespace+'/base_link', Time()).transform.rotation
            b2oMatrix = tf3d.quaternions.quat2mat(
                [b2oOrientation.w, b2oOrientation.x, b2oOrientation.y, b2oOrientation.z])[:3, :3]

            if self.b2pVector is None:
                # Offset to pressure sensor
                pressureOffset = self.tfBuffer.lookup_transform(
                    self.namespace+'/pressure_link', self.namespace+'/base_link', Time()).transform.translation
                self.b2pVector = [pressureOffset.x,
                                  pressureOffset.y, pressureOffset.z]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warning(str(ex))
            return

        # Rotate pressure sensor offset into odom frame and get additional depth from offset
        # TODO: Calculate uncertainty of this measure
        addedDepth = np.dot(b2oMatrix, self.b2pVector)[2]

        # Publish z offset from odom to base_link (depth)
        outMsg = PoseWithCovarianceStamped()
        outMsg.header = msg.header
        outMsg.header.frame_id = "odom"
        outMsg.pose.pose.position.z = msg.depth + addedDepth
        outMsg.pose.covariance[14] = msg.variance
        outMsg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(outMsg)


def main(args=None):
    rclpy.init(args=args)
    node = depthConverter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()