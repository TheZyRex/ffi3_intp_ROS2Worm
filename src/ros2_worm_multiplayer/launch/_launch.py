import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os

def generate_launch_description():
    package_name = 'ros2_worm_multiplayer'

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_terminal',
            default_value='false',
            description='Set to true to launch nodes in separate terminals'
        ),
        launch_ros.actions.Node(
            package='ros2_worm_multiplayer',
            executable='worm_navigation_node',
            name='navigation'
        ),
        launch_ros.actions.Node(
            package='ros2_worm_multiplayer',
            executable='worm_display_node',
            name='display'
        ),
        # Conditionally execute commands to open terminals if 'use_terminal' is set to true
        launch.actions.ExecuteProcess(
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_terminal')),
            cmd=[
                'gnome-terminal', '--', 'bash', '-i', '-c',
                'source ' + os.path.join(os.getcwd(), 'install', 'setup.bash') +
                ' && ros2 run ' + package_name + ' worm_display_node.py'
            ],
            name='open_terminal_node1',
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_terminal')),
            cmd=[
                'gnome-terminal', '--', 'bash', '-i', '-c',
                'source ' + os.path.join(os.getcwd(), 'install', 'setup.bash') +
                ' && ros2 run ' + package_name + ' worm_navigation_node'
            ],
            name='open_terminal_node2',
            output='screen'
        ),
    ])