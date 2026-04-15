import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    strategy_pkg_share = get_package_share_directory("oxebots_strategy")
    bringup_pkg_share = get_package_share_directory("oxebots_bringup")
    
    bringup_config_file = os.path.join(
        bringup_pkg_share, "config", "ssl_config.yaml"
    )

    declare_bt_xml_arg = DeclareLaunchArgument(
        "bt_xml",
        default_value="test_roles.xml",
        description="Behavior Tree XML file name",
    )
    bt_xml = LaunchConfiguration("bt_xml")

    # Path Planner Nodes
    path_planner_node_0 = Node(
        package="oxebots_strategy",
        executable="d_star_planner_node",
        name="d_star_planner_node_0",
        output="screen",
        parameters=[bringup_config_file, {"robot_id": 0}],
    )

    path_planner_node_1 = Node(
        package="oxebots_strategy",
        executable="d_star_planner_node",
        name="d_star_planner_node_1",
        output="screen",
        parameters=[bringup_config_file, {"robot_id": 1}],
    )

    path_planner_node_2 = Node(
        package="oxebots_strategy",
        executable="d_star_planner_node",
        name="d_star_planner_node_2",
        output="screen",
        parameters=[bringup_config_file, {"robot_id": 2}],
    )

    # Path Follower Nodes
    path_follower_node_0 = Node(
        package="oxebots_strategy",
        executable="movement_calculation_node",
        name="path_follower_node_0",
        output="screen",
        parameters=[bringup_config_file, {"robot_id": 0}],
    )

    path_follower_node_1 = Node(
        package="oxebots_strategy",
        executable="movement_calculation_node",
        name="path_follower_node_1",
        output="screen",
        parameters=[bringup_config_file, {"robot_id": 1}],
    )

    path_follower_node_2 = Node(
        package="oxebots_strategy",
        executable="movement_calculation_node",
        name="path_follower_node_2",
        output="screen",
        parameters=[bringup_config_file, {"robot_id": 2}],
    )

    # Strategy Node
    strategy_node = Node(
        package="oxebots_strategy",
        executable="strategy_node",
        name="strategy_node",
        output="screen",
        parameters=[
            bringup_config_file,
            {"bt_xml_path": bt_xml},
        ],
        cwd=strategy_pkg_share,
    )

    return LaunchDescription(
        [
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
