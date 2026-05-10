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

    # Argumento para escolher a árvore
    declare_bt_xml_arg = DeclareLaunchArgument(
        "bt_xml",
        default_value="master_strategy.xml",
        description="Behavior Tree XML file name",
    )

    # Helper to create nodes for each robot
    def create_robot_nodes(robot_id):
        planner = Node(
            package="oxebots_strategy",
            executable="d_star_planner_node",
            name=f"d_star_planner_node_{robot_id}",
            output="screen",
            parameters=[bringup_config_file, {"robot_id": robot_id}],
        )
        follower = Node(
            package="oxebots_strategy",
            executable="movement_calculation_node",
            name=f"path_follower_node_{robot_id}",
            output="screen",
            parameters=[bringup_config_file, {"robot_id": robot_id}],
        )
        strategy = Node(
            package="oxebots_strategy",
            executable="strategy_node",
            name=f"strategy_node_{robot_id}",
            output="screen",
            parameters=[
                bringup_config_file,
                {"robot_id": robot_id},
                {"bt_xml_path": LaunchConfiguration("bt_xml")}
            ],
            cwd=strategy_pkg_share,
        )
        return [planner, follower, strategy]

    # Create nodes for robots 0, 1, and 2
    all_nodes = []
    for i in range(3):
        all_nodes.extend(create_robot_nodes(i))

    return LaunchDescription([declare_bt_xml_arg] + all_nodes)