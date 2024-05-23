#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node

def generate_launch_description():
    ld = LaunchDescription()

    static_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0.15", "-0.06", "-0.018", "0.0", "0.0", "1.5008", "nerian_stereo_right_color_optical_frame", "lidar"])

    container = ComposableNodeContainer(
        name="blickfeld_qb2_component",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="blickfeld_qb2_ros2_driver",
                plugin="blickfeld::ros_interop::Qb2Driver",
                name="blickfeld_qb2_driver",
                parameters=[
                    {
                        "fqdn": "192.168.2.110",
                        "frame_id": "lidar",
                        "point_cloud_topic": "/bf/points_raw",
                        "use_measurement_timestamp": False,
                        "publish_intensity": False,
                        "publish_point_id": False,
                    }
                ],
            ),
        ],
        output="screen",
    )

    ld.add_action(static_tf)
    ld.add_action(container)

    return ld
