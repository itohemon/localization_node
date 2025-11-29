# launch/localization.launch.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node, LifecycleNode
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_file = os.path.join(
        os.environ["HOME"], "ros2_ws/src/localization_node/maps/my_map.yaml"
    )

    # Map Server (Lifecycle Node)
    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        parameters=[{"yaml_filename": map_file}],
    )

    # Map Server の configure (5秒後)
    configure_map_server = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/map_server", "configure"],
                output="screen",
            )
        ],
    )

    # Map Server の activate (6秒後)
    activate_map_server = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/map_server", "activate"],
                output="screen",
            )
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # Localization Node
    localization = Node(
        package="localization_node",
        executable="particle_localization_node",
        name="particle_localization_node",
        output="screen",
        parameters=[
            {
                "num_particles": 1500,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
            }
        ],
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory("particle_filter_localization"),
        "config",
        "particle_filter_localization.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_dir],
    )

    return LaunchDescription(
        [
            map_server,
            rviz,
            configure_map_server,
            activate_map_server,
            # static_tf,
            localization,
        ]
    )
