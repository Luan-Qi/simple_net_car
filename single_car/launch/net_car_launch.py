from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='127.0.0.1',
        description='IP address of the server'
    )

    return LaunchDescription([
        server_ip_arg,
        Node(
            package='single_car',
            executable='car_control_joy',
            name='car_control_joy',
            output='screen',
        ),
        Node(
            package='single_car',
            executable='car_control_socket',
            name='car_control_socket',
            output='screen',
            arguments=[LaunchConfiguration('server_ip'), '12345']
        ),
        Node(
            package='single_car',
            executable='video_server_tcp',
            name='video_server_tcp',
            output='screen',
        ),
    ])
