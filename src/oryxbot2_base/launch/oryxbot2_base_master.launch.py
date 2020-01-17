


import argparse
import os
import sys
import launch
import launch_ros.actions

from launch import LaunchDescription


def launch(launch_descriptor, argv):
    parser = argparse.ArgumentParser(description='launch oryxbot base demo')
    parser.add_argument(
        '--base',
        help='path to base')
    args = parser.parse_args(argv)

    ld = launch_descriptor

    #package = 'oryxbot2_base'
    #ld.add_process(
    #    cmd=[get_executable_path(package_name=package, executable_name='joy_node')],
    #    name='oryxbot2_base_node',
    #    exit_handler=restart_exit_handler,
    #)
    
    package = 'oryxbot2_kinematics'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='joy_node')],
        name='odometry2_node',
        exit_handler=restart_exit_handler,
    )
    
    package = 'oryxbot2_kinematics'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='joy_node')],
        name='oryxbot2_kinematic_node',
        exit_handler=restart_exit_handler,
    )

    package = 'oryxbot_joy'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='joy_node')],
        name='oryxbot2_joy_node',
        exit_handler=restart_exit_handler,
    )

    package = 'joy'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='joy_node')],
        name='joy_node',
        exit_handler=restart_exit_handler,
    )

    return ld


def main(argv=sys.argv[1:]):
    launcher = DefaultLauncher()
    launch_descriptor = launch(LaunchDescriptor(), argv)
    launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())














