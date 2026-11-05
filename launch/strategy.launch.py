import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    strategy_pkg_share = get_package_share_directory("oxebots_strategy")
    bringup_pkg_share = get_package_share_directory("oxebots_bringup")

    base_config = os.path.join(bringup_pkg_share, "config", "match_config.yaml")
    config_file = LaunchConfiguration("config_file").perform(context)
    if not config_file:
        config_file = base_config

    config_params = [base_config]
    if os.path.abspath(config_file) != os.path.abspath(base_config):
        config_params.append(config_file)

    bt_xml = LaunchConfiguration("bt_xml").perform(context)
    invert_sides = LaunchConfiguration("invert_sides").perform(context)
    require_gc = LaunchConfiguration("require_gc").perform(context)

    # Helper to create nodes for each robot
    def create_robot_nodes(robot_id):
        # We only add bt_xml_path if it's not empty to avoid overriding YAML
        strategy_parameters = [
            *config_params,
            {"robot_id": robot_id},
            {"invert_sides": invert_sides == "True"}
        ]

        if require_gc:
            strategy_parameters.append({"require_gc": require_gc.lower() in ("true", "1")})

        # Only override if bt_xml is explicitly provided
        if bt_xml:
            strategy_parameters.append({"bt_xml_path": bt_xml})

        planner = Node(
            package="oxebots_strategy",
            executable="d_star_planner_node",
            name=f"d_star_planner_node_{robot_id}",
            output="screen",
            parameters=[
                *config_params,
                {"robot_id": robot_id},
                {"invert_sides": invert_sides == "True"}
            ],
        )
        follower = Node(
            package="oxebots_strategy",
            executable="movement_calculation_node",
            name=f"path_follower_node_{robot_id}",
            output="screen",
            parameters=[
                *config_params,
                {"robot_id": robot_id},
                {"invert_sides": invert_sides == "True"}
            ],
        )
        trajectory_setpoint = Node(
            package="oxebots_strategy",
            executable="trajectory_setpoint_node",
            name=f"trajectory_setpoint_node_{robot_id}",
            output="screen",
            parameters=[*config_params, {"robot_id": robot_id}],
        )
        strategy = Node(
            package="oxebots_strategy",
            executable="strategy_node",
            name=f"strategy_node_{robot_id}",
            output="screen",
            parameters=strategy_parameters,
            cwd=strategy_pkg_share,
        )
        return [planner, follower, trajectory_setpoint, strategy]

    # Create nodes for robots 0, 1, and 2
    all_nodes = []
    for i in range(3):
        all_nodes.extend(create_robot_nodes(i))

    return all_nodes


def generate_launch_description():
    # Argumento para inverter os lados do campo (Simulador RA)
    declare_invert_sides_arg = DeclareLaunchArgument(
        "invert_sides",
        default_value="False",
        description="Invert field sides (for RA simulator)",
    )

    # Argumento para escolher a árvore
    declare_bt_xml_arg = DeclareLaunchArgument(
        "bt_xml",
        default_value="",
        description="Behavior Tree XML file name (overrides config if provided)",
    )

    # Argumento para o arquivo de configuração
    declare_config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="YAML configuration file path",
    )

    declare_require_gc_arg = DeclareLaunchArgument(
        "require_gc",
        default_value="",
        description="Override require_gc in strategy (True/False)",
    )

    return LaunchDescription([
        declare_invert_sides_arg, 
        declare_bt_xml_arg,
        declare_config_file_arg,
        declare_require_gc_arg,
        OpaqueFunction(function=launch_setup)
    ])