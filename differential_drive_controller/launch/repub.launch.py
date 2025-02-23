import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node




def generate_launch_description():
    ld = LaunchDescription()

    # wheel_base_config = LaunchConfiguration('wheel_base', default='0.17')
    # wheel_base = DeclareLaunchArgument(name='wheel_base', default_value=wheel_base_config, description='')

    # wheel_radius_config = LaunchConfiguration('wheel_radius', default='0.033')
    # wheel_radius = DeclareLaunchArgument(name='wheel_radius', default_value=wheel_radius_config, description='')

    # max_rpm_config = LaunchConfiguration('max_rpm', default='100')
    # max_rpm = DeclareLaunchArgument(name='max_rpm', default_value=max_rpm_config, description='')


    bot_launch_file = os.path.join(get_package_share_directory('bumperbot_description'), 'launch', 'gazebo.launch.py')

    bot_control_launch_file = os.path.join(get_package_share_directory('bumperbot_controller'), 'launch', 'diff_controller.launch.py')

    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bot_launch_file
        )
    )

    control_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bot_control_launch_file
        )
    )

    repub_node = Node(
        package='differential_drive_controller',
        executable='repub_node'
        # parameters=[{'wheel_base': wheel_base,
        #             'wheel_radius': wheel_radius,
        #             'max_rpm': max_rpm}]
    )

    # ld.add_action(wheel_base_config)
    # ld.add_action(wheel_base_config)
    # ld.add_action(wheel_radius_config)
    # ld.add_action(wheel_radius)
    # ld.add_action(max_rpm_config)
    # ld.add_action(max_rpm)

    ld.add_action(sim_cmd)
    ld.add_action(control_cmd)
    ld.add_action(repub_node)

    return ld