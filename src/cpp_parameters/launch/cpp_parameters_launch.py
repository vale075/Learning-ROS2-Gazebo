from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_parameters',
            executable='minimal_param_node',
            name='custom_minimal_param_node',
            output='screen', # to output to console
            emulate_tty=True, # to output to console
            parameters=[
                {'my_parameter': 'galaxy'}
            ]
        )
    ])