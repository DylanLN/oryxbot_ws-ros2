from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim', node_executable='turtlesim_node', output='screen'),
        # launch_ros.actions.Node(
        #     package='turtlesim', node_executable='turtle_teleop_key', output='screen'),

        launch_ros.actions.Node(
            package='tutorials_tf', node_executable='turtle_tf_broadcaster', arguments=['turtle2'], output='screen'),
        launch_ros.actions.Node(
            package='tutorials_tf', node_executable='turtle_tf_broadcaster', arguments=['turtle1'], output='screen'),
        launch_ros.actions.Node(
            package='tutorials_tf', node_executable='turtle_tf_listener', output='screen'),
    ])
