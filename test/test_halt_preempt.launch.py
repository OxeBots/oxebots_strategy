"""T06 — launch_testing: o HALT para o robô em até 300 ms (SC-02, R-05, critério 1 do Cap. 23).

Sobe processos reais (referee_state_node, movement_calculation_node e strategy_node) e faz o
próprio teste de "mundo": publica o árbitro (Referee), a visão (/game_data, /field_geometry) e a
rota do planejador (/robot_N/path), com um robô andando a ~1 m/s. Depois publica HALT e mede, em
/robot_commands, quanto tempo leva até o comando ficar zero e CONTINUAR zero.

Dois robôs, dois caminhos de segurança, no mesmo launch:
  * robô 0, SEM árvore: só o SafetyGate do controlador (regra 1, GameState HALT zera o comando).
    Prova que a segurança vale mesmo sem o strategy_node. O teste renova o /robot_goal durante o
    HALT: um objetivo novo não pode liberar o robô (R-05).
  * robô 1, COM árvore (behavior_trees/test_halt_preempt.xml): cadeia completa Referee ->
    referee_state_node -> strategy_node (HaltBranch, Halt -> MODE_HALT) e SafetyGate. Confere
    também o ramo publicado em /strategy/active_branch e o override MODE_HALT.

Ajustes (variáveis de ambiente, para mudar sem editar o arquivo):
  OXEBOTS_HALT_BUDGET_MS   limite de parada em ms (padrão 300; o regulamento dá 2 s)
  OXEBOTS_HALT_REPETITIONS ciclos RUN -> HALT por robô (padrão 10)
  OXEBOTS_HALT_ATTEMPTS    tentativas por ciclo inconclusivo (padrão 5, veja _cycle)
  OXEBOTS_TEST_DOMAIN_ID   ROS_DOMAIN_ID do teste (padrão 50 + PID % 50, isolado do grSim e das
                           partidas simuladas nos domínios 0 e 1)

Nada é medido sobre o robô físico: o robô simulado fica parado em /game_data, e é o controlador que
decide a velocidade do comando, que é o que o regulamento e o SafetyGate regulam.
"""

import math
import os
import statistics
import threading
import time
import unittest

# O domínio precisa estar definido ANTES de qualquer nó: os processos lançados herdam o ambiente.
os.environ['ROS_DOMAIN_ID'] = os.environ.get(
    'OXEBOTS_TEST_DOMAIN_ID', str(50 + os.getpid() % 50))
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from oxebots_interfaces.msg import (  # noqa: E402
    GameData, RobotCmd, RobotGameData, RobotGoal, RobotMotionOverride, SSLGeometryData)
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from ssl_league_msgs.msg import Referee  # noqa: E402
from std_msgs.msg import String  # noqa: E402

# ----------------------------------------------------------------------------------------------
# Parâmetros do teste
# ----------------------------------------------------------------------------------------------
HALT_BUDGET_MS = float(os.environ.get('OXEBOTS_HALT_BUDGET_MS', '300'))
REPETITIONS = int(os.environ.get('OXEBOTS_HALT_REPETITIONS', '10'))
ATTEMPTS = int(os.environ.get('OXEBOTS_HALT_ATTEMPTS', '5'))

ROBOT_GATE = 0   # só controlador + SafetyGate
ROBOT_TREE = 1   # strategy_node + controlador + SafetyGate

MIN_SPEED_BEFORE_HALT = 0.9   # m/s: o robô precisa estar mesmo andando (~1 m/s) antes do HALT
STOPPED_LINEAR = 0.01         # m/s
STOPPED_ANGULAR = 0.01        # rad/s
OBSERVE_AFTER_HALT_S = 1.2    # o comando precisa ficar zero por todo este tempo
MIN_ZERO_SAMPLES = 20         # e ter amostras zero suficientes (o controlador publica a 50 Hz)
# O controlador publica a cada 20 ms. Um buraco maior que isto nas amostras de um ciclo (ou o
# controlador mudo, ou o próprio processo do teste parado por falta de CPU) torna a medida
# inconclusiva: o ciclo é repetido em vez de reprovar ou aprovar às cegas.
MAX_SAMPLE_GAP_S = 0.15
PAUSE_AFTER_STALL_S = 1.0     # espera antes de repetir um ciclo em que o próprio teste travou

# Robôs parados na visão, longe da bola (não dispara a repulsão de contato) e fora das áreas.
ROBOT_POSE_M = {ROBOT_GATE: (-1.0, 0.5), ROBOT_TREE: (-1.0, -0.5)}
BALL_MM = (1000.0, 1000.0)
PATH_LENGTH_M = 1.2           # rota de 2 pontos: o controlador persegue o último (plena)


def generate_test_description():
    def controller(robot_id):
        return launch_ros.actions.Node(
            package='oxebots_strategy', executable='movement_calculation_node',
            name=f'path_follower_node_{robot_id}',
            parameters=[{'robot_id': robot_id}], output='screen')

    referee = launch_ros.actions.Node(
        package='oxebots_strategy', executable='referee_state_node', name='referee_state_node',
        parameters=[{'is_yellow_team': False, 'require_gc': True}], output='screen')

    strategy = launch_ros.actions.Node(
        package='oxebots_strategy', executable='strategy_node',
        name=f'strategy_node_{ROBOT_TREE}',
        parameters=[{
            'robot_id': ROBOT_TREE,
            'bt_xml_path': 'test_halt_preempt.xml',
            'is_yellow_team': False,
            'require_gc': True,
        }],
        cwd=get_package_share_directory('oxebots_strategy'),
        output='screen')

    return launch.LaunchDescription([
        referee,
        controller(ROBOT_GATE),
        controller(ROBOT_TREE),
        strategy,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class Sample:
    __slots__ = ('t', 'speed', 'omega', 'kick')

    def __init__(self, t, speed, omega, kick):
        self.t = t
        self.speed = speed
        self.omega = omega
        self.kick = kick

    def stopped(self):
        return (self.speed < STOPPED_LINEAR and self.omega < STOPPED_ANGULAR and self.kick == 0.0)


class World(Node):
    """Faz de GC, de visão e de planejador, e grava o que o controlador publica."""

    def __init__(self):
        super().__init__('halt_preempt_world')
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latched_log = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._lock = threading.Lock()
        self._command = Referee.COMMAND_HALT
        self._counter = 0
        self.samples = {ROBOT_GATE: [], ROBOT_TREE: []}
        self.branches = []     # (t, texto de /strategy/active_branch)
        self.overrides = []    # (t, robot_id, mode)
        self.timer_ticks = []  # instantes do timer de 60 Hz: mostram se o processo do teste travou

        self._game_data_pub = self.create_publisher(GameData, '/game_data', 10)
        self._referee_pub = self.create_publisher(
            Referee, '/gc_multicast_bridge/referee_messages', 10)
        self._geometry_pub = self.create_publisher(SSLGeometryData, '/field_geometry', latched)
        self._goal_pub = self.create_publisher(RobotGoal, '/robot_goal', latched)
        self._path_pubs = {
            rid: self.create_publisher(Path, f'/robot_{rid}/path', 10) for rid in ROBOT_POSE_M}

        self.create_subscription(
            RobotCmd, '/robot_commands', self._on_command, qos_profile_sensor_data)
        self.create_subscription(
            String, '/strategy/active_branch', self._on_branch, latched_log)
        self.create_subscription(
            RobotMotionOverride, '/robot_motion_override', self._on_override, 50)

        self.create_timer(1.0 / 60.0, self._publish_game_data)
        self.create_timer(0.1, self._publish_referee)
        self.create_timer(0.1, self._publish_gate_goal)
        self.create_timer(0.05, self._publish_paths)

        self._geometry_pub.publish(self._geometry())

    # --- publicação -------------------------------------------------------------------------
    @staticmethod
    def _geometry():
        msg = SSLGeometryData()
        msg.field.field_length = 4500
        msg.field.field_width = 3000
        msg.field.goal_width = 1000
        msg.field.penalty_area_depth = 500
        msg.field.penalty_area_width = 1350
        msg.field.center_circle_radius = 500
        return msg

    def _publish_game_data(self):
        with self._lock:
            self.timer_ticks.append(time.monotonic())
            if len(self.timer_ticks) > 20000:
                del self.timer_ticks[:10000]
        now = self.get_clock().now().to_msg()
        msg = GameData()
        msg.header.stamp = now
        for rid, (x, y) in ROBOT_POSE_M.items():
            robot = RobotGameData()
            robot.id = rid
            robot.x = x * 1000.0
            robot.y = y * 1000.0
            robot.orientation = 0.0
            msg.robots.allies.append(robot)
        msg.ball.x, msg.ball.y, msg.ball.z = BALL_MM[0], BALL_MM[1], 0.0
        msg.ball.confidence = 1.0
        msg.ball.stamp = now
        self._game_data_pub.publish(msg)

    def _referee_message(self):
        msg = Referee()
        msg.stage = 0
        msg.command = self._command
        msg.command_counter = self._counter
        msg.blue_team_on_positive_half = [False]
        for team in (msg.yellow, msg.blue):
            team.goalkeeper = 0
            team.can_place_ball = [False]
            team.max_allowed_bots = [3]
        return msg

    def _publish_referee(self):
        with self._lock:
            msg = self._referee_message()
        self._referee_pub.publish(msg)

    def _publish_gate_goal(self):
        # Robô sem árvore: este teste faz o papel dela e renova o objetivo (watchdog de 500 ms).
        # Continua publicando durante o HALT: um objetivo novo não pode liberar o robô.
        x, y = ROBOT_POSE_M[ROBOT_GATE]
        goal = RobotGoal()
        goal.robot_id = ROBOT_GATE
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y + PATH_LENGTH_M
        goal.pose.pose.orientation.w = 1.0
        goal.planner_type = RobotGoal.PLANNER_STRAIGHT_LINE
        goal.disable_arrival_latch = True
        self._goal_pub.publish(goal)

    def _publish_paths(self):
        # Faz o papel do D*: rota de 2 pontos para +y. O controlador persegue o primeiro ponto a
        # mais de 140 mm, que é o último: velocidade plena (min(1,0 m/s, 2 x distância)).
        now = self.get_clock().now().to_msg()
        for rid, (x, y) in ROBOT_POSE_M.items():
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = now
            for py in (y, y + PATH_LENGTH_M):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = py
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self._path_pubs[rid].publish(path)

    def send_command(self, command):
        """Publica o novo comando do árbitro; devolve o instante (monotônico) antes de enviar."""
        with self._lock:
            self._counter += 1
            self._command = command
            msg = self._referee_message()
        t_publish = time.monotonic()
        self._referee_pub.publish(msg)
        return t_publish

    # --- gravação ---------------------------------------------------------------------------
    def _on_command(self, msg):
        now = time.monotonic()
        with self._lock:
            for cmd in msg.robots:
                if cmd.id in self.samples:
                    self.samples[cmd.id].append(Sample(
                        now, math.hypot(cmd.x_velocity, cmd.y_velocity),
                        abs(cmd.angular_velocity), cmd.kick_speed))

    def _on_branch(self, msg):
        with self._lock:
            self.branches.append((time.monotonic(), msg.data))

    def _on_override(self, msg):
        with self._lock:
            self.overrides.append((time.monotonic(), msg.robot_id, msg.mode))

    # --- consulta ---------------------------------------------------------------------------
    def samples_since(self, robot_id, t0):
        with self._lock:
            return [s for s in self.samples[robot_id] if s.t >= t0]

    def moving_fast(self, robot_id, count=15):
        with self._lock:
            tail = self.samples[robot_id][-count:]
        return (len(tail) == count and time.monotonic() - tail[-1].t < 0.2 and
                all(s.speed >= MIN_SPEED_BEFORE_HALT for s in tail))

    def branches_since(self, t0):
        with self._lock:
            return [text for t, text in self.branches if t >= t0]

    def timer_gap(self, t_start, t_end):
        """Maior intervalo sem disparo do timer de 60 Hz em [t_start, t_end].

        Se o processo do teste travou (disco ou CPU ocupados), o timer também parou: o buraco nas
        amostras é do ambiente. Se o timer andou e as amostras não, o controlador é que ficou mudo.
        """
        with self._lock:
            ticks = [x for x in self.timer_ticks if t_start <= x <= t_end]
        return largest_gap_of_times([t_start] + ticks + [t_end])

    def overrides_since(self, robot_id, t0):
        with self._lock:
            return [mode for t, rid, mode in self.overrides if t >= t0 and rid == robot_id]


def wait_until(predicate, timeout_s, period_s=0.02):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()


def stop_analysis(samples, t0):
    """Latência até o comando ficar zero e continuar zero até o fim da observação.

    Devolve (latência_s, amostras_zero_no_final, duração_observada_s). A latência é None se o robô
    nunca parou ou se o último comando observado ainda era de movimento.
    """
    last_moving = -1
    for index, sample in enumerate(samples):
        if not sample.stopped():
            last_moving = index
    tail = samples[last_moving + 1:]
    duration = samples[-1].t - t0 if samples else 0.0
    if not tail:
        return None, 0, duration
    return tail[0].t - t0, len(tail), duration


def largest_gap_of_times(times):
    return max(later - earlier for earlier, later in zip(times, times[1:]))


def largest_gap(samples, t_start, t_end):
    """Maior intervalo sem amostra em [t_start, t_end] (inclui as pontas)."""
    return largest_gap_of_times([t_start] + [s.t for s in samples] + [t_end])


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1)]


class TestHaltPreempt(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.world = World()
        cls.inconclusive_cycles = 0
        # SingleThreadedExecutor DE PROPÓSITO: com o MultiThreadedExecutor do rclpy (Python) a
        # carga deste teste (~250 callbacks/s: timers mais /robot_commands dos dois controladores a
        # 50 Hz cada) satura o laço do executor e os timers pausam por até 2,7 s. Medido: 126 de
        # 720 ticks em 12 s, contra 720 de 720 com uma thread.
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.world)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        # Deixa o robô em HALT ao terminar.
        cls.world.send_command(Referee.COMMAND_HALT)
        cls.executor.shutdown()
        cls.world.destroy_node()
        rclpy.shutdown()

    def _cycle(self, robot_id):
        """RUN com o robô a ~1 m/s -> HALT. Devolve (latência_s, t0).

        Se as amostras do ciclo tiverem um buraco maior que MAX_SAMPLE_GAP_S, há dois casos:
          * o timer do próprio teste também parou: o processo do teste travou (disco ou CPU
            ocupados). A medida é inconclusiva e o ciclo é repetido, no máximo ATTEMPTS vezes;
          * o timer andou e o controlador não publicou: o controlador ficou mudo depois do HALT.
            É defeito do produto e reprova na hora, sem repetir.
        """
        world = self.world
        gap = 0.0
        world_gap = 0.0
        for attempt in range(1, ATTEMPTS + 1):
            world.send_command(Referee.COMMAND_FORCE_START)
            self.assertTrue(
                wait_until(lambda: world.moving_fast(robot_id), timeout_s=20.0),
                f'o robô {robot_id} não chegou a {MIN_SPEED_BEFORE_HALT} m/s em RUN: '
                'o teste não mede nada se o robô não estiver andando')

            t0 = world.send_command(Referee.COMMAND_HALT)
            time.sleep(OBSERVE_AFTER_HALT_S)
            t_end = time.monotonic()
            samples = world.samples_since(robot_id, t0)

            gap = largest_gap(samples, t0, t_end)
            if gap <= MAX_SAMPLE_GAP_S:
                break
            world_gap = world.timer_gap(t0, t_end)
            if world_gap <= MAX_SAMPLE_GAP_S:
                self.fail(
                    f'robô {robot_id}: o controlador ficou {gap * 1000:.0f} ms sem publicar em '
                    '/robot_commands depois do HALT, e o processo do teste não travou '
                    f'(maior pausa do timer: {world_gap * 1000:.0f} ms)')
            self.inconclusive_cycles += 1
            print(
                f'[HALT-LATENCY] robô {robot_id}: ciclo inconclusivo, o processo do teste travou '
                f'(pausa de {world_gap * 1000:.0f} ms; limite {MAX_SAMPLE_GAP_S * 1000:.0f} ms), '
                f'tentativa {attempt}/{ATTEMPTS}', flush=True)
            time.sleep(PAUSE_AFTER_STALL_S)
        else:
            self.fail(
                f'robô {robot_id}: o processo do teste travou (pausas de até '
                f'{world_gap * 1000:.0f} ms) em {ATTEMPTS} tentativas seguidas. A máquina está '
                'ocupada demais para medir; veja /proc/pressure/io e /proc/pressure/cpu e rode de '
                'novo com ela parada')

        latency, zeros, _ = stop_analysis(samples, t0)
        self.assertIsNotNone(
            latency, f'robô {robot_id}: o comando ainda era de movimento no fim da observação')
        self.assertGreaterEqual(
            zeros, MIN_ZERO_SAMPLES,
            f'robô {robot_id}: poucas amostras zero depois de parar ({zeros})')
        return latency, t0

    def _report(self, label, latencies_ms):
        print(
            f'[HALT-LATENCY] {label}: n={len(latencies_ms)} '
            f'mediana={statistics.median(latencies_ms):.0f} ms '
            f'p95={percentile(latencies_ms, 0.95):.0f} ms max={max(latencies_ms):.0f} ms '
            f'(limite {HALT_BUDGET_MS:.0f} ms; ciclos inconclusivos repetidos até agora: '
            f'{self.inconclusive_cycles})', flush=True)

    def test_00_processes_are_up(self):
        names = {'referee_state_node', f'path_follower_node_{ROBOT_GATE}',
                 f'path_follower_node_{ROBOT_TREE}', f'strategy_node_{ROBOT_TREE}'}

        def all_up():
            return names.issubset(set(self.world.get_node_names()))

        self.assertTrue(wait_until(all_up, timeout_s=30.0), f'nós ausentes: {names}')

    def test_01_safety_gate_halts_without_the_tree(self):
        latencies = []
        for _ in range(REPETITIONS):
            latency, _ = self._cycle(ROBOT_GATE)
            latencies.append(latency * 1000.0)
        self._report('SafetyGate sem árvore (robô 0)', latencies)
        self.assertLessEqual(
            max(latencies), HALT_BUDGET_MS,
            f'o HALT levou {max(latencies):.0f} ms (limite {HALT_BUDGET_MS:.0f} ms)')

    def test_02_full_chain_halts_with_the_tree(self):
        latencies = []
        for cycle in range(REPETITIONS):
            latency, t0 = self._cycle(ROBOT_TREE)
            latencies.append(latency * 1000.0)

            # A árvore decidiu pelo HaltBranch e pediu MODE_HALT ao controlador.
            self.assertTrue(
                any(f'robot_id={ROBOT_TREE} branch=HaltBranch' in text
                    for text in self.world.branches_since(t0)),
                f'ciclo {cycle}: /strategy/active_branch não registrou a ida para HaltBranch')
            self.assertIn(
                RobotMotionOverride.MODE_HALT, self.world.overrides_since(ROBOT_TREE, t0),
                f'ciclo {cycle}: a árvore não pediu MODE_HALT em /robot_motion_override')
        self._report('cadeia completa com árvore (robô 1)', latencies)
        self.assertLessEqual(
            max(latencies), HALT_BUDGET_MS,
            f'o HALT levou {max(latencies):.0f} ms (limite {HALT_BUDGET_MS:.0f} ms)')

    def test_03_robot_moves_again_after_halt(self):
        # Regressão "preso em HALT": FORCE_START depois do HALT tem de liberar os dois robôs.
        world = self.world
        world.send_command(Referee.COMMAND_HALT)
        time.sleep(0.5)
        world.send_command(Referee.COMMAND_FORCE_START)
        for robot_id in (ROBOT_GATE, ROBOT_TREE):
            self.assertTrue(
                wait_until(lambda rid=robot_id: world.moving_fast(rid), timeout_s=10.0),
                f'o robô {robot_id} não voltou a andar depois do FORCE_START')
