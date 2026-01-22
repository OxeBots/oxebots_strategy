import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    strategy_pkg_share = get_package_share_directory("oxebots_strategy")
    behavior_tree_path = os.path.join(strategy_pkg_share, "test_tree.xml")

    # Argumento para a cor do time (herdado do ssl.launch.py ou definido aqui)
    declare_is_yellow_arg = DeclareLaunchArgument(
        'is_yellow',
        default_value='false',
        description='Whether the team is yellow (true) or blue (false)'
    )
    is_yellow = LaunchConfiguration('is_yellow')

    # --- Parâmetros comuns para todos os nós de movimento ---
    common_movement_params = [
        {'max_linear_speed': 0.5},
        {'p_gain_linear': 0.5},
        {'is_yellow': is_yellow}, # Passa o parâmetro is_yellow
    ]


    # Executa o nó de estratégia
    strategy_node = Node(
        package="oxebots_strategy",
        executable="strategy_node",
        name="strategy_node",
        output="screen",
        parameters=[
            {"bt_xml_path": behavior_tree_path},
            {"is_yellow": is_yellow} # Passa o parâmetro is_yellow para o strategy_node também
        ],
        cwd=strategy_pkg_share,
    )

    # Retorna a lista de todos os nós que devem ser lançados
    return LaunchDescription([
        declare_is_yellow_arg,
        strategy_node
    ])
