import launch
import xacro
import traceback
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration as LC
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace


def get_zed_description(zed_name, zed_type, robot, context, debug):
    zed_model_path = PathJoinSubstitution([
        get_package_share_directory('zed_wrapper'),
        'urdf',
        'zed_descr.urdf.xacro'
    ]).perform(context)

    zed_description_data = xacro.process_file(zed_model_path, mappings={
        'debug': debug,
        'namespace': robot,
        'inertial_reference_frame': 'world',
        'camera_name': f"{robot}/{zed_name}",
        'camera_model': zed_type
    }).toxml()

    zed_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=f"{zed_name}",
        name='zed_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': zed_description_data},
            {'use_tf_static': True}
        ]
    )

    return zed_state_publisher


def check_zed_xacro(context):
    use_zed_camera = False
    zed_xacro_path = ''
    try:
        zed_xacro_path = PathJoinSubstitution([
            get_package_share_directory('zed_wrapper'),
            'urdf',
            'zed_descr.urdf.xacro'
        ]).perform(context)

        use_zed_camera = True
    except:
        print("zed_wrapper not found. Launching without zed TF")

    return (str(use_zed_camera), zed_xacro_path)


def evaluate_xacro(context, *args, **kwargs):
    robot = LC('robot').perform(context)
    debug = False

    robot_xacro_path = PathJoinSubstitution([
        get_package_share_directory('mercury_descriptions'),
        robot,
        'xacro',
        robot + '.xacro'
    ]).perform(context)

    (use_zed_camera, zed_xacro_path) = check_zed_xacro(context)

    sim_enabled = LC('sim_enabled').perform(context)

    try:
        robot_description_data = xacro.process_file(
            robot_xacro_path,
            mappings={
                'namespace': robot,
                'use_zed_camera': use_zed_camera,
                'zed_xacro_path': zed_xacro_path,
                'sim_enabled' : sim_enabled
            }
        ).toxml()

        robot_state_publisher = Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[
                {'robot_description': robot_description_data},
                {'use_tf_static': True}
            ]
        )

        nodes = [robot_state_publisher]

        try:
            nodes.append(get_zed_description("ffc", "zedxm", robot, context, debug))

            nodes.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="ffc_to_zed_tf",
                    arguments=[
                        "0", "0", "0",
                        "0", "0", "0",
                        f"{robot}/ffc_base_link",
                        f"{robot}/ffc_camera_link",
                    ],
                )
            )

        except PackageNotFoundError:
            print("zed_wrapper not found. Launching without zed TF")

        return nodes

    except Exception as e:
        print()
        print("---------------------------------------------")
        print("COULD NOT OPEN ROBOT DESCRIPTION OR ZED XACRO FILE")
        print(e)
        traceback.print_exc()
        print("---------------------------------------------")
        print()

    return []


def launch_ekf(context, *args, **kwargs):
    launch_items = []

    if LC("ekf_enabled").perform(context) != "True":
        return launch_items

    robot = LC("robot").perform(context)
    tag_odom_enabled = LC("tag_odom_enabled").perform(context) == 'True'
    ekf_config_name = f"{robot}_ekf.yaml" if not tag_odom_enabled else f"{robot}_ekf_tag_odom.yaml"

    config = os.path.join(
        get_package_share_directory('mercury_hardware'),
        'config',
        ekf_config_name
    )

    # start robot_localization Extended Kalman filter (EKF)
    launch_items.append(
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_localization_node',
            output='screen',
            parameters=[
                config,
                {
                    'reset_on_time_jump': True,
                }
            ]
        )
    )

    # start tag odom
    if tag_odom_enabled:
        launch_items.append(
            Node(
                package='mercury_hardware',
                executable='tag_odom.py',
                name='tag_odom',
                output='screen'
            )
        )

    return launch_items


def generate_launch_description():
    return launch.LaunchDescription([
        # Read in the vehicle's namespace through the command line or use the default value one is not provided
        DeclareLaunchArgument(
            "robot",
            default_value="default_robot",
            description="Namespace of the vehicle",
        ),

        DeclareLaunchArgument(
            "ekf_enabled",
            default_value="True",
            description="Enable EKF to estimate robot odometry"
        ),

        DeclareLaunchArgument(
            "tag_odom_enabled",
            default_value="False",
            description="Enable navigation using the apriltag"
        ),

        DeclareLaunchArgument(
            "sim_enabled",
            default_value="False",
            description="Enable sim to read proper xacro"
        ),

        GroupAction([
            PushRosNamespace(
                LC("robot")
            ),

            # Publish world and odom as same thing until we get SLAM
            # This is here so we can compare ground truth from sim to odom
            Node(
                name="odom_to_world_broadcaster",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "odom"]
            ),

            Node(
                package='mercury_hardware',
                executable='depth_converter.py',
                name='depth_converter',
            ),

            # start ekf
            OpaqueFunction(function=launch_ekf),

            # Publish robot model for Sensor locations
            OpaqueFunction(function=evaluate_xacro),
        ], scoped=True)
    ])