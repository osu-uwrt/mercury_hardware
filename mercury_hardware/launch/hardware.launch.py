from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import GroupAction, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression
from launch.conditions.if_condition import IfCondition

import os

micro_ros_agent_launch_file = os.path.join(
    get_package_share_directory('mercury_hardware'),
    "launch", "micro_ros_agent.launch.py",
)

imu_launch_file = os.path.join(
    get_package_share_directory('mercury_imu'),
    "launch", "imu.launch.py"
)

# apriltag_launch_file = os.path.join(
#     get_package_share_directory('mercury_hardware'),
#     "launch", "apriltag.launch.py"
# )

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value="mercury",
                              description="Name of the vehicle"),
        
        GroupAction([
            PushRosNamespace(
                LC("robot")
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(micro_ros_agent_launch_file),
                launch_arguments=[
                    ('robot', LC('robot')),
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(imu_launch_file),
                launch_arguments=[
                    ('robot', LC('robot'))
                ]
            ),
            # IncludeLaunchDescription(
            #     AnyLaunchDescriptionSource(apriltag_launch_file),
            #     launch_arguments=[
            #         ('robot', LC('robot')),
            #     ]
            # ),
        ], scoped=True)
    ])
