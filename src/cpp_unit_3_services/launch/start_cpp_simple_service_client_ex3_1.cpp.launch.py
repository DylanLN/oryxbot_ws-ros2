"""Launch cpp_simple_service_client_node_ex3_1"""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([ launch_ros.actions.Node(
        package='cpp_unit_3_services',
        node_executable='cpp_simple_service_client_ex3_1_node',
        output='screen'),
    ])
