#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    container = ComposableNodeContainer(
        name="blickfeld_qb2_component",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                package="blickfeld_qb2_ros2_driver",
                plugin="blickfeld::ros_interop::Qb2SnapshotDriver",
                name="blickfeld_qb2_snapshot_driver",
                parameters=[
                    {
                        # HINT: ros2 doesn't allow passing empty lists.
                        #     "unix_sockets": [],
                        #     "unix_socket_frame_ids": [],
                        #     "unix_socket_point_cloud_topics": [],
                        "fqdns": ["qb2_1", "qb2_2"],
                        "fqdn_frame_ids": ["qb2_1", "qb2_2"],
                        "fqdn_point_cloud_topics": ["/bf/points_raw_1", "/bf/points_raw_2"],
                        "snapshot_frequency": 0.1,
                        "use_measurement_timestamp": False,
                        "publish_intensity": True,
                        "publish_point_id": True,
                    }
                ],
                remappings=[
                    ("trigger_snapshot", "/bf/trigger_snapshot"),
                ],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )
    return LaunchDescription([container])
