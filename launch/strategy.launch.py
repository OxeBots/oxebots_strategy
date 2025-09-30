from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oxebots_strategy',
            executable='strategy_node',
            name='strategy_node',
            output='screen',
            parameters=[{'robot_amount': 3}],
        )
    ])
