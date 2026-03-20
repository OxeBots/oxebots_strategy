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
        default_value="test_tree.xml",
        description="Behavior Tree XML file name (e.g., test_tree.xml or simple_attack.xml)",
    )

    is_yellow = LaunchConfiguration("is_yellow")
    bt_xml = LaunchConfiguration("bt_xml")

    common_movement_params = [
        {"max_linear_speed": 1.0},
        {"p_gain_linear": 1.5},
        {"is_yellow": is_yellow},
    ]

    path_planner_node_0 = Node(
        package="oxebots_strategy",
        executable="d_star_planner_node",
        name="d_star_planner_node_0",
        output="screen",
        parameters=common_movement_params + [{"robot_id": 0}],
    )

    path_planner_node_1 = Node(
        package="oxebots_strategy",
        executable="d_star_planner_node",
        name="d_star_planner_node_1",
        output="screen",
        parameters=common_movement_params + [{"robot_id": 1}],
    )

    path_planner_node_2 = Node(
        package="oxebots_strategy",
        executable="d_star_planner_node",
        name="d_star_planner_node_2",
        output="screen",
        parameters=common_movement_params + [{"robot_id": 2}],
    )

    path_follower_node_0 = Node(
        package="oxebots_strategy",
        executable="movement_calculation_node",
        name="path_follower_node_0",
        output="screen",
        parameters=common_movement_params + [{"robot_id": 0}],
    )

    path_follower_node_1 = Node(
        package="oxebots_strategy",
        executable="movement_calculation_node",
        name="path_follower_node_1",
        output="screen",
        parameters=common_movement_params + [{"robot_id": 1}],
    )

    path_follower_node_2 = Node(
        package="oxebots_strategy",
        executable="movement_calculation_node",
        name="path_follower_node_2",
        output="screen",
        parameters=common_movement_params + [{"robot_id": 2}],
    )

    strategy_node = Node(
        package="oxebots_strategy",
        executable="strategy_node",
        name="strategy_node",
        output="screen",
        parameters=[
            {"bt_xml_path": bt_xml},
            {"is_yellow": is_yellow},
        ],
        cwd=strategy_pkg_share,
    )

    return LaunchDescription(
        [
            declare_is_yellow_arg,
            declare_bt_xml_arg,
            path_planner_node_0,
            path_planner_node_1,
            path_planner_node_2,
            path_follower_node_0,
            path_follower_node_1,
            path_follower_node_2,
            strategy_node,
        ]
    )
