import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

# from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Encontra o caminho para a pasta de compartilhamento do seu pacote de estratégia
    strategy_pkg_share = get_package_share_directory("oxebots_strategy")

    # Define o caminho completo para o arquivo XML da árvore de comportamento
    behavior_tree_path = os.path.join(strategy_pkg_share, "test_tree.xml")

    return LaunchDescription(
        [
            Node(
                package="oxebots_strategy",
                executable="movement_calculation_node",
                name="movement_calculation_node",
                output="screen",
                parameters=[
                    {'max_linear_speed': 2.0},
                    {'p_gain_linear': 0.5}
                ]
            ),
            # 3. Executa o nó de estratégia (strategy_node)
            Node(
                package="oxebots_strategy",
                executable="strategy_node",
                name="strategy_node",
                output="screen",
                # Passa o caminho do arquivo da árvore de comportamento como parâmetro
                parameters=[{"bt_xml_path": behavior_tree_path}],
                cwd=strategy_pkg_share,
            )
        ]
    )
