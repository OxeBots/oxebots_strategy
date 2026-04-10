import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    strategy_pkg_share = get_package_share_directory("oxebots_strategy")

    declare_is_yellow_arg = DeclareLaunchArgument(
        "is_yellow",
        default_value="false",
        description="Whether the team is yellow (true) or blue (false)",
    )

    declare_bt_xml_arg = DeclareLaunchArgument(
        "bt_xml",
        default_value="test_roles.xml",
        description="Behavior Tree XML file name (e.g., test_roles.xml or simple_attack.xml)",
    )

    is_yellow = LaunchConfiguration("is_yellow")
    bt_xml = LaunchConfiguration("bt_xml")

    common_movement_params = [
        {"max_linear_speed": 1.0},
        {"p_gain_linear": 1.5},
        {"is_yellow": is_yellow},
    ]

    # Planners e Followers para os 3 robôs
    nodes = []
    for i in range(3):
        nodes.append(Node(
            package="oxebots_strategy",
            executable="d_star_planner_node",
            name=f"d_star_planner_node_{i}",
            output="screen",
            parameters=common_movement_params + [{"robot_id": i}],
        ))
        nodes.append(Node(
            package="oxebots_strategy",
            executable="movement_calculation_node",
            name=f"path_follower_node_{i}",
            output="screen",
            parameters=common_movement_params + [{"robot_id": i}],
        ))
        # Nó de Estratégia individual para cada robô
        nodes.append(Node(
            package="oxebots_strategy",
            executable="strategy_node",
            name=f"strategy_node_{i}",
            output="screen",
            parameters=[
                {"bt_xml_path": bt_xml},
                {"is_yellow": is_yellow},
                {"robot_id": i},
            ],
            cwd=strategy_pkg_share,
        ))

    # Nó de Alocação de Papéis (Centralizado)
    role_assigner_node = Node(
        package="oxebots_strategy",
        executable="role_assigner_node",
        name="role_assigner_node",
        output="screen",
        parameters=[{"goalkeeper_id": 0}],
    )

    return LaunchDescription(
        [
            declare_is_yellow_arg,
            declare_bt_xml_arg,
            role_assigner_node,
            *nodes
        ]
    )
