# oxebots_strategy

ROS 2 strategy and path tracking package for OxeBots.

## Nós Principais

### `movement_calculation_node`
Nó responsável por receber o caminho planejado (`/robot_<id>/path`) e calcular as velocidades lineares e angulares omnidirecionais ($v_x, v_y, \omega$) publicadas em `/robot_commands`.

#### Algoritmo de Seguimento
* **Pure Pursuit Omnidirecional com Ganho P:**
  * O robô calcula um ponto alvo (*lookahead target*) ao longo da lista de poses da rota.
  * O vetor de velocidade linear é proporcional à distância e direção até esse ponto.
* **Orientação e Controle Angular:**
  * Se houver um comando prioritário em `/robot_goal`, o robô alinha a orientação angular com esse alvo.
  * Se não houver alvo prioritário, o nó extrai a orientação quaternária da própria rota (`pose.orientation`) e aplica controle proporcional de yaw.

#### Parâmetros Configuráveis
| Parâmetro | Tipo | Padrão | Descrição |
| :--- | :--- | :--- | :--- |
| `lookahead_distance` | `double` | `100.0` | Distância de perseguição em milímetros. Calibrado para coincidir com 2 células da grade D* (resolução de 50 mm), filtrando o efeito escada sem cortar curvas. |
| `max_linear_speed` | `double` | `3.0` | Limite superior da velocidade linear em m/s. |
| `p_gain_linear` | `double` | `2.0` | Ganho proporcional da velocidade linear. |
| `max_angular_speed`| `double` | `4.0` | Velocidade angular máxima em rad/s. |
| `p_gain_angular` | `double` | `3.0` | Ganho proporcional do alinhamento angular. |
| `angle_tolerance` | `double` | `0.1` | Tolerância angular em radianos para zerar o comando de giro. |
