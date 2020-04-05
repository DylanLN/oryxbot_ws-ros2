from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='rplidar_ros', 
            node_executable='rplidarNode_A1', 
            parameters=[{'serial_port': '/dev/ttyUSB0',
                            'serial_baudrate': 115200,
                            'frame_id': 'laser_radar_Link',
                            'inverted': False,
                            'angle_compensate': True,
                            'output_angle_min': -180,
                            'output_angle_max': 180,}],
            output='screen'),]
    )
