#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf2_ros
import transforms3d as tf3d


def pose_to_matrix(p: Pose) -> np.ndarray:
    q = [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
    T = np.eye(4)
    T[:3, :3] = tf3d.quaternions.quat2mat(q)
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def matrix_to_pose(T: np.ndarray) -> Pose:
    p = Pose()
    p.position.x = float(T[0, 3])
    p.position.y = float(T[1, 3])
    p.position.z = float(T[2, 3])

    q = tf3d.quaternions.mat2quat(T[:3, :3])  # [w, x, y, z]
    p.orientation.w = float(q[0])
    p.orientation.x = float(q[1])
    p.orientation.y = float(q[2])
    p.orientation.z = float(q[3])
    return p


def transform_to_matrix(transform) -> np.ndarray:
    q = [
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
    ]
    T = np.eye(4)
    T[:3, :3] = tf3d.quaternions.quat2mat(q)
    T[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return T


def rotate_covariance_6x6(cov_flat, R):
    """Rotate a 6x6 covariance [pos, rot] or [lin, ang] by 3x3 R."""
    cov = np.array(cov_flat, dtype=float).reshape(6, 6)
    X = np.zeros((6, 6))
    X[:3, :3] = R
    X[3:, 3:] = R
    cov_out = X @ cov @ X.T
    return cov_out.reshape(-1).tolist()


class ZedOdomToBase(Node):
    def __init__(self):
        super().__init__("zed_odom_to_base")

        self.declare_parameter("input_topic", "/mercury/ffc/zed_node/odom")
        self.declare_parameter("output_topic", "/mercury/ffc/zed_node/odom_base")
        self.declare_parameter("base_frame", "mercury/base_center")
        self.declare_parameter("camera_frame", "mercury/ffc_camera_link")
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("overwrite_twist_covariance", False)
        self.declare_parameter("twist_variance_linear", 0.05)
        self.declare_parameter("twist_variance_angular", 0.05)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.overwrite_twist_covariance = self.get_parameter(
            "overwrite_twist_covariance"
        ).value
        self.twist_var_linear = float(
            self.get_parameter("twist_variance_linear").value
        )
        self.twist_var_angular = float(
            self.get_parameter("twist_variance_angular").value
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            Odometry, self.output_topic, qos_profile_sensor_data
        )
        self.sub = self.create_subscription(
            Odometry, self.input_topic, self.odom_cb, qos_profile_sensor_data
        )

        # 180 deg yaw: flips X and Y, keeps Z
        self.R_flip_xy = np.diag([-1.0, -1.0, 1.0])
        self.T_flip_xy = np.eye(4)
        self.T_flip_xy[:3, :3] = self.R_flip_xy

        self.get_logger().info(
            f"Republishing {self.input_topic} -> {self.output_topic} as {self.base_frame} odom "
            "with X/Y sign flip applied"
        )

    def odom_cb(self, msg: Odometry):
        try:
            # source -> target, so this is T_base_camera
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as ex:
            self.get_logger().warning(f"TF lookup failed: {ex}")
            return

        T_base_camera = transform_to_matrix(tf_msg.transform)
        R_base_camera = T_base_camera[:3, :3]

        # Raw ZED pose is world->camera in a camera-aligned world basis.
        T_world_camera_raw = pose_to_matrix(msg.pose.pose)

        # Fixed basis remap: camera-world axes -> base-world axes.
        # For your +90 deg roll mount, this is the camera->base rotation.
        T_fix = np.eye(4)
        T_fix[:3, :3] = R_base_camera

        # First remap the WORLD basis, then convert tracked body from camera -> base.
        T_world_camera_fixed = T_fix @ T_world_camera_raw
        T_camera_base = np.linalg.inv(T_base_camera)
        T_world_base = T_world_camera_fixed @ T_camera_base

        # Blunt patch: flip X and Y signs on the final output transform.
        T_world_base = self.T_flip_xy @ T_world_base

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.world_frame
        out.child_frame_id = self.base_frame
        out.pose.pose = matrix_to_pose(T_world_base)

        # Rotate pose covariance into the fixed basis, then apply X/Y flip.
        pose_cov = rotate_covariance_6x6(msg.pose.covariance, R_base_camera)
        out.pose.covariance = rotate_covariance_6x6(
            pose_cov, self.R_flip_xy
        )

        # Twist in Odometry is expressed in child_frame_id.
        # Convert camera-body twist into base-body twist, including lever-arm correction.
        p_base_camera = T_base_camera[:3, 3]

        v_cam = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
        w_cam = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ])

        w_base = R_base_camera @ w_cam
        v_cam_in_base = R_base_camera @ v_cam

        # v_camera = v_base + w_base x p_base_camera
        # => v_base = v_camera - w_base x p_base_camera
        v_base = v_cam_in_base - np.cross(w_base, p_base_camera)

        # Apply X/Y sign flip to twist as requested.
        v_base = self.R_flip_xy @ v_base
        w_base = self.R_flip_xy @ w_base

        out.twist.twist.linear.x = float(v_base[0])
        out.twist.twist.linear.y = float(v_base[1])
        out.twist.twist.linear.z = float(v_base[2])
        out.twist.twist.angular.x = float(w_base[0])
        out.twist.twist.angular.y = float(w_base[1])
        out.twist.twist.angular.z = float(w_base[2])

        if self.overwrite_twist_covariance:
            cov = [0.0] * 36
            cov[0] = self.twist_var_linear
            cov[7] = self.twist_var_linear
            cov[14] = self.twist_var_linear
            cov[21] = self.twist_var_angular
            cov[28] = self.twist_var_angular
            cov[35] = self.twist_var_angular
            out.twist.covariance = cov
        else:
            twist_cov = rotate_covariance_6x6(
                msg.twist.covariance, R_base_camera
            )
            out.twist.covariance = rotate_covariance_6x6(
                twist_cov, self.R_flip_xy
            )

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ZedOdomToBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()