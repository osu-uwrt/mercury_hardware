from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.actions.composable_node_container import ComposableNode, ComposableNodeContainer

cfg_36h11 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.508,
    "max_hamming": 0,
    "z_up": True
}

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        name="robot",
        default_value="mercury",
        description="name of the robot"
    ))

    ld.add_action(ComposableNodeContainer(
        name='tag_container',
        namespace="apriltag",
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'error'],
        composable_node_descriptions=[
            ComposableNode(
                name='apriltag_36h11',
                package='apriltag_ros', plugin='AprilTagNode',
                remappings=[
                    ("image_rect", "ffc/zed_node/left/color/rect/image"),
                    ("camera_info", "ffc/zed_node/left/color/rect/camera_info"),
                ],
                parameters=[cfg_36h11],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen'
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='surface_frame_node',
        arguments=["0", "0.4572", "0", "0", "-1.5707", "-1.5707",
                   "tag36h11:0", "estimated_origin_frame"]
    ))

    return ld
