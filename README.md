# oxebots_strategy

Pacote central de tomada de decisão, alocação de papéis, planejamento e controle de trajetórias da equipe **OxeBots** (UFBA) para a categoria RoboCup Small Size League – Entry Level (SSL-EL).

---

## 1. Visão Geral da Arquitetura

O pacote `oxebots_strategy` é estruturado em nós desacoplados que cooperam para controlar os robôs em campo:

```text
[ Árbitro & Visão ] ──> role_assigner_node ──> [ /role_assignment ]
                              │
[ game_data / ball_predicted ] ──> strategy_node (Behavior Tree 4.x)
                                          │
                                   [ /robot_<id>/goal ]
                                          │
    [ /map ] ──────────────────────> d_star_planner_node
                                          │
                                   [ /robot_<id>/path ]
                                          │
    ┌─────────────────────────────────────┴─────────────────────────────────────┐
    │ (Modo Velocidade Omnidirecional)          │ (Modo Trajetória NRF24)      │
    ▼                                           ▼                               │
movement_calculation_node                  trajectory_setpoint_node             │
    │                                           │                               │
[ /robot_commands ]                     [ /robot_trajectory_setpoints ]        │
    ▼                                           ▼                               │
oxebots_comms (grSim / RA)                 oxebots_comms (NRF24 via bitproto)  │
```

---

## 2. Nós do Pacote

### 2.1 `strategy_node`
Executa o motor de árvore de comportamento (**BehaviorTree.CPP v4**) para um robô específico em tempo real (60 Hz).
- Lê o estado filtrado de jogo (`game_data`, `ball_predicted`, `robot_predicted`), avalia comandos do árbitro e gera os alvos táticos para o robô.
- **Tópicos Assinados:**
  - `game_data` (`oxebots_interfaces/msg/GameData`)
  - `ball_predicted` (`oxebots_interfaces/msg/BallPredicted`)
  - `robot_predicted` (`oxebots_interfaces/msg/RobotPredicted`)
  - `/role_assignment` (`oxebots_interfaces/msg/RoleAssignment`)
  - `/gc_multicast_bridge/referee_messages` (`ssl_ros_bridge/msg/SSLRefereeData`)
- **Tópicos Publicados:**
  - `/robot_<id>/goal` (`oxebots_interfaces/msg/RobotGoal`)
  - `/robot_motion_override` (`oxebots_interfaces/msg/RobotMotionOverride`)
- **Parâmetros:**
  - `robot_id` (int, default: `1`): ID do robô controlado por esta instância.
  - `is_yellow_team` (bool, default: `false`): `true` se o time for Amarelo; `false` se Azul.
  - `invert_sides` (bool, default: `false`): Inverte sentido do ataque.
  - `bt_xml_path` (string, default: `""`): Caminho do arquivo XML da árvore (ex: `master_strategy_carry.xml`).
  - `execution_rate` (double, default: `60.0` Hz): Taxa de execução de ticks da Behavior Tree.
  - `possession_distance` (double, default: `220.0` mm): Raio máximo para considerar que o robô tem posse da bola.

### 2.2 `role_assigner_node`
Distribui dinamicamente os papéis entre os robôs de linha com base no estado do jogo, custo geométrico até a bola e histerese temporal para evitar oscilações.
- **Tópicos Assinados:** `game_data` (`oxebots_interfaces/msg/GameData`).
- **Tópicos Publicados:** `/role_assignment` (`oxebots_interfaces/msg/RoleAssignment`).
- **Parâmetros:**
  - `goalkeeper_id` (int, default: `0`): ID fixado para o goleiro.
  - `w1` (double, default: `2.0`): Peso atribuído à distância linear até a bola.
  - `w2` (double, default: `1.0`): Peso atribuído ao alinhamento angular com o gol adversário.
  - `hysteresis` (double, default: `2.5`): Bônus de histerese para manter o atacante atual.
  - `time_hysteresis` (double, default: `2.0` s): Tempo mínimo antes de autorizar nova troca de papel.
  - `is_yellow_team` (bool, default: `false`).

### 2.3 `d_star_planner_node`
Planejador de caminhos online baseado no algoritmo D* Lite. Encontra caminhos livres de obstáculos considerando o mapa de custos e obstáculos dinâmicos (outros robôs e bola).
- **Tópicos Assinados:**
  - `map` (`nav_msgs/msg/OccupancyGrid`)
  - `/robot_<id>/goal` (`oxebots_interfaces/msg/RobotGoal`)
  - `game_data` (`oxebots_interfaces/msg/GameData`)
- **Tópicos Publicados:**
  - `/robot_<id>/path` (`nav_msgs/msg/Path`)
- **Parâmetros:**
  - `robot_id` (int, default: `0`)
  - `planning_rate_hz` (double, default: `10.0` Hz): Taxa de replanejamento normal.
  - `planning_rate_hz_close` (double, default: `40.0` Hz): Taxa acelerada quando próximo da bola.
  - `close_ball_distance_m` (double, default: `0.4` m): Limite de proximidade para aumentar a taxa.
  - `robot_safety_radius` (double, default: `0.20` m): Raio de proteção contra colisões com outros robôs.
  - `ball_safety_radius` (double, default: `0.15` m): Raio de proteção para não tocar na bola indevidamente.
  - `occupancy_threshold` (int, default: `80`): Custo a partir do qual uma célula é tratada como obstáculo.
  - `max_expansions` (int, default: `100000`): Limite de nós expandidos por busca.

### 2.4 `movement_calculation_node`
Seguidor de caminho baseado em perseguição pura (*Pure Pursuit*) omnidirecional com controle proporcional de velocidade e PD de orientação.
- **Tópicos Assinados:**
  - `/robot_<id>/path` (`nav_msgs/msg/Path`)
  - `/robot_<id>/goal` (`oxebots_interfaces/msg/RobotGoal`)
  - `game_data` (`oxebots_interfaces/msg/GameData`)
  - `/robot_motion_override` (`oxebots_interfaces/msg/RobotMotionOverride`)
- **Tópicos Publicados:**
  - `robot_commands` (`oxebots_interfaces/msg/RobotCmdData`)
  - `/robot_<id>/motion_status` (`oxebots_interfaces/msg/RobotMotionStatus`)
- **Parâmetros:**
  - `lookahead_distance` (double, default: `140.0` mm): Distância ao longo do caminho para extração do ponto-alvo.
  - `max_linear_speed` (double, default: `1.0` m/s): Velocidade linear máxima permitida.
  - `p_gain_linear` (double, default: `2.0`): Ganho proporcional para translação.
  - `max_angular_speed` (double, default: `3.0` rad/s): Velocidade angular máxima permitida.
  - `p_gain_angular` (double, default: `2.2`): Ganho proporcional de orientação.
  - `d_gain_angular` (double, default: `0.25`): Ganho derivativo de orientação.
  - `angle_tolerance` (double, default: `0.04` rad): Zona morta angular.

### 2.5 `trajectory_setpoint_node`
Nó intermediário para envio eficiente de trajetórias aos robôs físicos via rádio NRF24 (pacote compacto `bitproto`).
- **Tópicos Assinados:**
  - `/robot_<id>/path` (`nav_msgs/msg/Path`)
  - `/robot_<id>/goal` (`oxebots_interfaces/msg/RobotGoal`)
  - `game_data` (`oxebots_interfaces/msg/GameData`)
- **Tópicos Publicados:**
  - `robot_trajectory_setpoints` (`oxebots_interfaces/msg/RobotTrajectorySetpointData`)
- **Parâmetros:**
  - `robot_id` (int, default: `0`)
  - `radio_publish_rate_hz` (double, default: `15.0` Hz)
  - `lookahead_safety_factor` (double, default: `1.5`)
  - `max_lookahead_mm` (double, default: `1000.0` mm)

---

## 3. Pré-requisitos e Dependências

- **Sistema:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy Jalisco
- **Bibliotecas:** `BehaviorTree.CPP` (v4), `tf2_ros`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `oxebots_interfaces`, `ssl_ros_bridge`.

---

## 4. Licença

Este pacote é disponibilizado sob a licença **GPLv3**.
