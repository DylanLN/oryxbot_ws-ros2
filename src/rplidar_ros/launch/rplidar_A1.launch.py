from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([

        # serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
        # serial_baudrate = LaunchConfiguration('serial_baudrate', default=115200)
        # frame_id = LaunchConfiguration('frame_id', default='laser_radar_Link')
        # inverted = LaunchConfiguration('inverted', default=False)
        # angle_compensate = LaunchConfiguration('angle_compensate', default=True)
        # output_angle_min = LaunchConfiguration('output_angle_min', default=-180)
        # output_angle_max = LaunchConfiguration('output_angle_max', default=180)


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

            # laser_parameters = {
            #     'serial_port': '/dev/ttyUSB0',
            #     'serial_baudrate': 115200,
            #     'frame_id': 'laser_radar_Link',
            #     'inverted': False,
            #     'angle_compensate': True,
            #     'output_angle_min': -180,
            #     'output_angle_max': 180,
            # }