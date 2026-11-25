# oxebots_strategy

Pacote central de tomada de decisão, alocação de papéis, planejamento e controle de trajetórias da equipe **OxeBots** (UFBA) para a categoria RoboCup Small Size League – Entry Level (SSL-EL).

---

## 1. Visão Geral da Arquitetura

O pacote `oxebots_strategy` é estruturado em nós desacoplados que cooperam para controlar os robôs em campo:

```text
[ Árbitro & Bola ] ─────> referee_state_node ──> [ /game_state ]
                                                       │
[ game_data ] ──────────> role_assigner_node ──> [ /role_assignment ]
                                                       │
[ game_data / /game_state / /field_geometry ] ──> strategy_node (Behavior Tree 4.x)
                                                       │
                                                [ /robot_goal ]
                                                       │
    [ /map / /game_state ] ──────────────────> d_star_planner_node
                                                       │
                                              [ /robot_<id>/path ]
                                                       │
                                           movement_calculation_node
                                                       │
                                              [ /robot_commands ]
                                                       │
    ┌──────────────────────────────────────────────────┴──────────────────────┐
    │ (Simuladores / Rádio em modo velocity)            (Rádio em modo point) │
    ▼                                                                         ▼
oxebots_comms (grSim / RA / NRF24)                              trajectory_setpoint_node
                                                                              │
                                                               [ /robot_trajectory_setpoints ]
                                                                              ▼
                                                                oxebots_comms (NRF24 bitproto)
```

---

## 2. Nós do Pacote

### 2.1 `strategy_node`
Executa o motor de árvore de comportamento (**BehaviorTree.CPP v4**) para um robô específico em tempo real (60 Hz).
- Lê o estado consolidado de jogo (`game_data`, `/game_state`, `/field_geometry`, `/role_assignment`), atualiza a blackboard e gera os alvos táticos e overrides de movimento para o robô.
- **Tópicos Assinados:**
  - `game_data` (`oxebots_interfaces/msg/GameData`)
  - `/game_state` (`oxebots_interfaces/msg/GameState`)
  - `/role_assignment` (`oxebots_interfaces/msg/RoleAssignment`)
  - `/gc_multicast_bridge/referee_messages` (`ssl_league_msgs/msg/Referee`)
  - `/field_geometry` (`oxebots_interfaces/msg/SSLGeometryData`)
- **Tópicos Publicados (pelo nó e suas folhas):**
  - `/robot_goal` (`oxebots_interfaces/msg/RobotGoal`)
  - `/robot_motion_override` (`oxebots_interfaces/msg/RobotMotionOverride`)
  - `/robot_motion_limit` (`std_msgs/msg/Float32`)
  - `/strategy/active_branch` (`std_msgs/msg/String`)
- **Parâmetros:**
  - `robot_id` (int, default: `1`): ID do robô controlado por esta instância.
  - `is_yellow_team` (bool, default: `false`): `true` se o time for Amarelo; `false` se Azul.
  - `invert_sides` (bool, default: `false`): Espelha coordenadas do campo.
  - `bt_xml_path` (string, default: `""`): Caminho do arquivo XML da árvore (ex.: `master_strategy_carry.xml`).
  - `execution_rate` (double, default: `60.0` Hz): Taxa de execução de ticks da Behavior Tree.
  - `game_state_topic` (string, default: `"/game_state"`): Tópico de estado consolidado do árbitro.
  - `gc_topic` (string, default: `"/gc_multicast_bridge/referee_messages"`): Tópico com mensagens brutas do Game Controller.
  - `possession_distance` (double, default: `220.0` mm): Raio máximo para considerar que o adversário controla a bola.
  - `require_gc` (bool, default: `true`): Exige mensagens frescas do Game Controller na condição `IsDataFresh`.
  - `max_gc_ms` (double, default: `500.0` ms): Idade máxima tolerada para dados do árbitro.
  - `max_pose_ms` (double, default: `150.0` ms): Idade máxima tolerada para a pose do próprio robô.
  - `max_ball_ms` (double, default: `300.0` ms): Idade máxima tolerada para os dados da bola.
  - `slot_margin_mm` (double, default: `150.0` mm): Margem extra de segurança usada pela folha `ComputeLegalSlot`.
  - `absolute_max_mps` (double, default: `1.5` m/s): Teto absoluto de velocidade aplicado pela folha `SetMotionLimit`.
  - `max_linear_speed` (double, default: `1.0` m/s): Velocidade linear máxima usada pelos cálculos táticos (ex.: `CalculateInterception`, `LateralClear`, `IsBallClosest`).
  - `min_linear_speed` (double, default: `0.0` m/s): Piso mínimo de velocidade linear (`0.0` desativa; sempre limitado pelo teto máximo).
  - `max_angular_speed` (double, default: `3.0` rad/s): Velocidade angular máxima configurada no nó.
  - `min_angular_speed` (double, default: `0.0` rad/s): Piso mínimo de velocidade angular (`0.0` desativa; sempre limitado pelo teto máximo).

### 2.2 `referee_state_node`
Interpreta os pacotes do Game Controller oficial, resolve o contexto de reinício (`NORMAL_START` de kick-off vs. pênalti), detecta entrada da bola em jogo (`PHASE_RUN`), calcula o lado defensivo (`our_goal_positive_x`) e monitora expiração (`stale`).
- **Tópicos Assinados:**
  - `/gc_multicast_bridge/referee_messages` (`ssl_league_msgs/msg/Referee`)
  - `ball_data` (`oxebots_interfaces/msg/BallPosition`)
- **Tópicos Publicados:**
  - `/game_state` (`oxebots_interfaces/msg/GameState`, QoS `transient_local`)
- **Parâmetros:**
  - `is_yellow_team` (bool, default: `false`)
  - `gc_topic` (string, default: `"/gc_multicast_bridge/referee_messages"`)
  - `ball_topic` (string, default: `"ball_data"`)
  - `stale_timeout_ms` (double, default: `500.0` ms): Tempo máximo sem mensagens do GC antes de marcar `stale = true`.
  - `ball_moved_mm` (double, default: `50.0` mm): Deslocamento da bola que encerra um reinício e transita para `PHASE_RUN`.
  - `kick_timeout_s` (double, default: `10.0` s): Prazo padrão de cobrança caso o GC não publique `current_action_time_remaining`.
  - `stop_speed_mps` (double, default: `0.75` m/s): Teto de velocidade linear em `PHASE_STOP`.
  - `play_speed_mps` (double, default: `1.5` m/s): Teto de velocidade linear nas fases de jogo.
  - `require_gc` (bool, default: `true`): Define se a ausência inicial ou perda do GC ativa o estado `stale`.

### 2.3 `role_assigner_node`
Distribui dinamicamente os papéis entre os robôs de linha com base no estado do jogo, custo geométrico até a bola, congelamento com bola em voo e histerese temporal para evitar oscilações.
- **Tópicos Assinados:** `game_data` (`oxebots_interfaces/msg/GameData`).
- **Tópicos Publicados:** `/role_assignment` (`oxebots_interfaces/msg/RoleAssignment`).
- **Parâmetros:**
  - `goalkeeper_id` (int, default: `0`): ID fixado para o goleiro.
  - `w1` (double, default: `2.0`): Peso atribuído à distância linear até a bola.
  - `w2` (double, default: `1.0`): Peso atribuído ao alinhamento cinético com o gol adversário.
  - `hysteresis` (double, default: `2.5`): Bônus de histerese para manter o atacante atual.
  - `time_hysteresis` (double, default: `2.0` s): Tempo mínimo antes de autorizar nova troca de papel.
  - `is_yellow_team` (bool, default: `false`).
  - `ball_flight_speed_mps` (double, default: `1.0` m/s): Velocidade da bola acima da qual ela é considerada em voo (congela troca de papéis; `0.0` desativa).
  - `receive_distance_mm` (double, default: `300.0` mm): Distância do candidato à bola abaixo da qual o congelamento de voo é encerrado imediatamente.

### 2.4 `d_star_planner_node`
Planejador de caminhos online baseado no algoritmo D* Lite (com suporte também ao modo `straight_line`). Encontra caminhos livres de obstáculos considerando o mapa de custos, áreas de defesa, zonas de bola parada e obstáculos dinâmicos (outros robôs e bola).
- **Tópicos Assinados:**
  - `map` (`nav_msgs/msg/OccupancyGrid`)
  - `/robot_goal` (`oxebots_interfaces/msg/RobotGoal`)
  - `game_data` (`oxebots_interfaces/msg/GameData`)
  - `/field_geometry` (`oxebots_interfaces/msg/SSLGeometryData`)
  - `/game_state` (`oxebots_interfaces/msg/GameState`)
- **Tópicos Publicados:**
  - `/robot_<id>/path` (`nav_msgs/msg/Path`)
  - `/robot_<id>/path_viz` (`nav_msgs/msg/Path`)
- **Parâmetros:**
  - `robot_id` (int, default: `0`)
  - `is_yellow_team` (bool, default: `false`)
  - `invert_sides` (bool, default: `false`)
  - `planning_rate_hz` (double, default: `10.0` Hz): Taxa de replanejamento normal.
  - `planning_rate_hz_close` (double, default: `40.0` Hz): Taxa acelerada quando próximo da bola.
  - `close_ball_distance_m` (double, default: `0.4` m): Limite de proximidade para aumentar a taxa.
  - `robot_safety_radius` (double, default: `0.20` m): Raio de proteção contra colisões com outros robôs.
  - `ball_safety_radius` (double, default: `0.15` m): Raio de proteção para não tocar na bola indevidamente.
  - `occupancy_threshold` (int, default: `80`): Custo a partir do qual uma célula é tratada como obstáculo.
  - `max_expansions` (int, default: `100000`): Limite de nós expandidos por busca.

### 2.5 `movement_calculation_node`
Seguidor de caminho baseado em perseguição pura (*Pure Pursuit*) omnidirecional (50 Hz) com controle proporcional de velocidade, PD de orientação, proteção dinâmica de área e **SafetyGate** (teto de velocidade por fase, clamp de chute em 3,0 m/s e watchdog de 500 ms). É o único escritor de `/robot_commands`.
- **Tópicos Assinados:**
  - `/robot_<id>/path` (`nav_msgs/msg/Path`)
  - `/robot_goal` (`oxebots_interfaces/msg/RobotGoal`)
  - `/game_data` (`oxebots_interfaces/msg/GameData`)
  - `/robot_motion_override` (`oxebots_interfaces/msg/RobotMotionOverride`)
  - `/field_geometry` (`oxebots_interfaces/msg/SSLGeometryData`)
  - `/game_state` (`oxebots_interfaces/msg/GameState`)
  - `/robot_motion_limit` (`std_msgs/msg/Float32`)
- **Tópicos Publicados:**
  - `/robot_commands` (`oxebots_interfaces/msg/RobotCmd`)
  - `/robot_motion_status` (`oxebots_interfaces/msg/RobotMotionStatus`)
- **Parâmetros:**
  - `robot_id` (int, default: `0`)
  - `is_yellow_team` (bool, default: `false`)
  - `invert_sides` (bool, default: `false`)
  - `path_renews_watchdog` (bool, default: `false`): Se `true`, pacotes de rota também renovam o watchdog de comando.
  - `lookahead_distance` (double, default: `140.0` mm): Distância ao longo do caminho para extração do ponto-alvo.
  - `robot_collision_distance` (double, default: `0.30` m): Distância mínima entre centros aplicada pelo filtro de velocidade a aliados e adversários; atua até o dobro dessa distância, inclusive em rotas retas. `0.0` desativa esse filtro. Quando ativo, prioriza afastamento e pode zerar comandos abaixo do piso mínimo configurado para preservar a separação.
  - `max_linear_speed` (double, default: `1.0` m/s): Velocidade linear máxima padrão.
  - `min_linear_speed` (double, default: `0.0` m/s): Piso mínimo de velocidade linear para comandos não-nulos (`0.0` desativa; nunca excede o teto máximo vigente nem sobrescreve HALT/chegada/watchdog).
  - `p_gain_linear` (double, default: `2.0`): Ganho proporcional para translação.
  - `max_angular_speed` (double, default: `3.0` rad/s): Velocidade angular máxima permitida.
  - `min_angular_speed` (double, default: `0.0` rad/s): Piso mínimo de velocidade angular para comandos não-nulos (`0.0` desativa; nunca excede o teto angular vigente).
  - `p_gain_angular` (double, default: `2.2`): Ganho proporcional de orientação.
  - `d_gain_angular` (double, default: `0.25`): Ganho derivativo de orientação.
  - `angle_tolerance` (double, default: `0.04` rad): Zona morta angular.

### 2.6 `trajectory_setpoint_node`
Nó intermediário que projeta a última velocidade decidida pelo `movement_calculation_node` e a pose estimada pelo Kalman em setpoints de trajetória para envio aos robôs físicos via rádio NRF24 no modo `point` (pacote compacto `bitproto`).
- **Tópicos Assinados:**
  - `robot_predicted` (`oxebots_interfaces/msg/RobotPrediction`)
  - `/robot_goal` (`oxebots_interfaces/msg/RobotGoal`)
  - `/robot_commands` (`oxebots_interfaces/msg/RobotCmd`)
- **Tópicos Publicados:**
  - `/robot_trajectory_setpoints` (`oxebots_interfaces/msg/RobotTrajectorySetpoint`)
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
