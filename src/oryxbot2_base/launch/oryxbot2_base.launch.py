import launch_ros.actions

from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='joy',
            node_executable='joy_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='oryxbot2_kinematics',
            node_executable='odometry2_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='oryxbot2_kinematics',
            node_executable='oryxbot2_kinematic_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='oryxbot_joy',
            node_executable='oryxbot2_joy_node',
            output='screen'
        ),
    ])
