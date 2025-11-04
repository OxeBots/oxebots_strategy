import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # --- Configuração dos Caminhos (igual ao seu) ---
    strategy_pkg_share = get_package_share_directory("oxebots_strategy")
    behavior_tree_path = os.path.join(strategy_pkg_share, "test_tree.xml")

    # --- Parâmetros comuns para todos os nós de movimento ---
    common_movement_params = [
        {'max_linear_speed': 0.5},
        {'p_gain_linear': 0.5}
    ]

    # --- INÍCIO DA CORREÇÃO ---
    # Lançar um "cérebro" de movimento (Jogador) para CADA robô

    # Nó de movimento para o Robô 0
    movement_node_0 = Node(
        package="oxebots_strategy",
        executable="movement_calculation_node",
        name="movement_calculation_node_0",  # Nome único
        output="screen",
        parameters=common_movement_params + [{'robot_id': 0}] # Passa o ID 0
    )

    # Nó de movimento para o Robô 1
    movement_node_1 = Node(
        package="oxebots_strategy",
        executable="movement_calculation_node",
        name="movement_calculation_node_1",  # Nome único
        output="screen",
        parameters=common_movement_params + [{'robot_id': 1}] # Passa o ID 1
    )

    # Nó de movimento para o Robô 2
    movement_node_2 = Node(
        package="oxebots_strategy",
        executable="movement_calculation_node",
        name="movement_calculation_node_2",  # Nome único
        output="screen",
        parameters=common_movement_params + [{'robot_id': 2}] # Passa o ID 2
    )

    # --- FIM DA CORREÇÃO ---

    # Executa o nó de estratégia (Técnico) - (igual ao seu)
    strategy_node = Node(
        package="oxebots_strategy",
        executable="strategy_node",
        name="strategy_node",
        output="screen",
        parameters=[{"bt_xml_path": behavior_tree_path}],
        cwd=strategy_pkg_share,
    )

    # Retorna a lista de todos os nós que devem ser lançados
    return LaunchDescription([
        movement_node_0,
        movement_node_1,
        movement_node_2,
        strategy_node
    ])
