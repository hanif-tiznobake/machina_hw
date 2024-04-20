from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():    
    server_freq = 500
    client_freq = 200
    
    return LaunchDescription([
        Node(
            package='sensor_service',
            executable='service',
            name='server_1',
            namespace='sensor_service',
            output='screen',
            parameters=[
                { 'sensor_name': 'sensor_1' },
                { 'sensor_ip': '127.0.0.3' },
                { 'sensor_port': 10000 },
                { 'frequency': server_freq }
            ]
        ),
        Node(
            package='sensor_service',
            executable='service',
            name='server_2',
            namespace='sensor_service',
            output='screen',
            parameters=[
                { 'sensor_name': 'sensor_2' },
                { 'sensor_ip': '127.0.0.4' },
                { 'sensor_port': 10000 },
                { 'frequency': server_freq }
            ]
        ),
        Node(
            package='sensor_service',
            executable='client',
            name='client',
            namespace='sensor_service',
            output='screen',
            parameters=[
                { 'server_name_1': 'sensor_1' },
                { 'server_name_2:' 'sensor_2' },
                { 'frequency': client_freq }
            ]
        )
    ])
