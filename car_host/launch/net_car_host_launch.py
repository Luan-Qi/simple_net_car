from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='127.0.0.1',
        description='IP address of the server'
    )

    joy_send_node = Node(
        package='car_host',
        executable='joy_send_socket',
        output='screen',
        arguments=['--server_ip', LaunchConfiguration('server_ip'), 
                   '--server_port', '12345']
    )

    tcp_address = PythonExpression([
        '"tcp://" + "',  # 注意嵌套引号的技巧
        LaunchConfiguration('server_ip'),
        '" + ":12321"'
    ])

    video_rev_node = ExecuteProcess(
        # ffplay -fflags nobuffer -flags low_delay -vf setpts=PTS-STARTPTS tcp://服务器IP:12345
        cmd=['ffplay', 
             '-fflags', 'nobuffer', 
             '-flags', 'low_delay', 
             '-vf','setpts=PTS-STARTPTS', 
             tcp_address],
        output='screen'
    )

    return LaunchDescription([
        server_ip_arg,
        joy_send_node,
        video_rev_node
    ])