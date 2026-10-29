#!/usr/bin/env python3
"""
Análise de mensagens do Game Controller gravadas em rosbag (backlog T03, hipótese H-01).

Responde: o que o campo `command` do Referee faz entre um reinício (kick-off, pênalti, free kick)
e o próximo STOP/HALT? O regulamento descreve o estado "Run", mas não diz que `command` o
Game Controller publica durante ele. Este script lê uma bag (do `log2bag` ou de `ros2 bag
record`) e mostra, com evidência:

  1. a linha do tempo de comandos/estágios (só as mudanças);
  2. cada reinício: quanto tempo o `command` permaneceu, para qual comando passou, e como se
     comportou o `current_action_time_remaining` (o prazo de 10 s do reinício);
  3. anomalias: `command` que mudou SEM um novo `command_counter` (o GC mudando de comando por
     conta própria);
  4. a cobertura: quais comandos aparecem e quais faltam na gravação;
  5. (só com --log, que traz o tracker da visão) em quanto tempo a bola se move ≥ 50 mm depois
     de cada NORMAL_START/DIRECT_FREE, comparado ao prazo do reinício;
  6. um parecer automático sobre o H-01.

A entrada pode ser uma rosbag (precisa do ROS) ou um log oficial da liga (.log ou .log.gz),
lido direto, SEM ROS (precisa de python3-protobuf e do compilador `protoc`).

Uso:

  python3 scripts/analisar_gc_bag.py --log jogo.log.gz     # log oficial da liga, sem ROS
  python3 scripts/analisar_gc_bag.py caminho/da/bag        # rosbag (source install/setup.bash)
  python3 scripts/analisar_gc_bag.py caminho/da/bag --topic /gc_multicast_bridge/referee_messages
  python3 scripts/analisar_gc_bag.py caminho/da/bag --csv linha_do_tempo.csv
  python3 scripts/analisar_gc_bag.py --selftest      # testa a lógica com dados sintéticos

Tópicos aceitos (detectados automaticamente): /referee_messages (gerado pelo log2bag) e
/gc_multicast_bridge/referee_messages (gravado ao vivo).
"""

import argparse
from collections import Counter, namedtuple
import csv
import gzip
import math
from pathlib import Path
import statistics
import struct
import subprocess
import sys
import tempfile

COMMANDS = {
    0: 'HALT', 1: 'STOP', 2: 'NORMAL_START', 3: 'FORCE_START',
    4: 'PREPARE_KICKOFF_YELLOW', 5: 'PREPARE_KICKOFF_BLUE',
    6: 'PREPARE_PENALTY_YELLOW', 7: 'PREPARE_PENALTY_BLUE',
    8: 'DIRECT_FREE_YELLOW', 9: 'DIRECT_FREE_BLUE',
    10: 'INDIRECT_FREE_YELLOW', 11: 'INDIRECT_FREE_BLUE',
    12: 'TIMEOUT_YELLOW', 13: 'TIMEOUT_BLUE',
    14: 'GOAL_YELLOW', 15: 'GOAL_BLUE',
    16: 'BALL_PLACEMENT_YELLOW', 17: 'BALL_PLACEMENT_BLUE',
}
STAGES = {
    0: 'NORMAL_FIRST_HALF_PRE', 1: 'NORMAL_FIRST_HALF', 2: 'NORMAL_HALF_TIME',
    3: 'NORMAL_SECOND_HALF_PRE', 4: 'NORMAL_SECOND_HALF', 5: 'EXTRA_TIME_BREAK',
    6: 'EXTRA_FIRST_HALF_PRE', 7: 'EXTRA_FIRST_HALF', 8: 'EXTRA_HALF_TIME',
    9: 'EXTRA_SECOND_HALF_PRE', 10: 'EXTRA_SECOND_HALF', 11: 'PENALTY_SHOOTOUT_BREAK',
    12: 'PENALTY_SHOOTOUT', 13: 'POST_GAME',
}
# Comandos que dão início a um reinício ou a jogo corrido (os que o H-01 investiga).
RESTART_COMMANDS = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11}
GOAL_COMMANDS = {14, 15}  # legado: o GC emite 1 mensagem e volta ao HALT com o mesmo contador
RUNNING_COMMANDS = {2, 3, 8, 9, 10, 11}  # comandos sob os quais o jogo corre
# Cobertura esperada de uma gravação completa (gols e legados são opcionais).
EXPECTED_COMMANDS = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 16, 17}
PROTO_DIR = (Path(__file__).resolve().parents[2] / 'ssl_ros_bridge' / 'ssl_league_protobufs'
             / 'proto')
PROTO_FILES = ('ssl_gc_referee_message.proto', 'ssl_gc_common.proto',
               'ssl_gc_game_event.proto', 'ssl_gc_geometry.proto',
               'ssl_vision_wrapper_tracked.proto', 'ssl_vision_detection_tracked.proto')
LOG_TRACKER = 5  # MESSAGE_SSL_VISION_TRACKER_2020
BALL_MOVED_MM = 50.0  # regra 5.4: a bola entra em jogo ao se mover pelo menos 0,05 m
LOG_MAGIC = b'SSL_LOG_FILE'
LOG_REFEREE = 3  # MESSAGE_SSL_REFBOX_2013
TOPICS = ('/referee_messages', '/gc_multicast_bridge/referee_messages')

Record = namedtuple('Record', [
    't', 'stage', 'command', 'counter', 'next_command', 'action_time_left', 'stage_time_left',
    'designated', 'blue_positive'])


def cmd_name(code):
    return COMMANDS.get(code, f'?{code}')


def seconds(value):
    return None if value is None else value / 1e6


# ------------------------------------------------------------------ leitura da bag (ROS)

def load_records(path, topic=None):
    """Lê a bag e devolve uma lista de Record (tempo em segundos desde a 1ª mensagem)."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        sys.exit("Não achei o rosbag2_py/rclpy (faça 'source install/setup.bash' num ambiente "
                 f'ROS 2): {exc}')

    reader = None
    for storage in ('', 'sqlite3', 'mcap'):
        try:
            candidate = rosbag2_py.SequentialReader()
            candidate.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                           rosbag2_py.ConverterOptions('', ''))
            reader = candidate
            break
        except Exception:  # noqa: BLE001 - tenta o próximo formato de armazenamento
            continue
    if reader is None:
        sys.exit(f'Não consegui abrir a bag em {path}')

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    chosen = topic or next((t for t in TOPICS if t in types), None)
    if chosen is None or chosen not in types:
        sys.exit(f'Tópico do árbitro não encontrado. Tópicos na bag: {sorted(types)}')
    msg_cls = get_message(types[chosen])

    records, t0 = [], None
    while reader.has_next():
        name, data, stamp_ns = reader.read_next()
        if name != chosen:
            continue
        msg = deserialize_message(data, msg_cls)
        t0 = stamp_ns if t0 is None else t0
        designated = (msg.designated_position[0].x, msg.designated_position[0].y) \
            if len(msg.designated_position) else None
        records.append(Record(
            t=(stamp_ns - t0) / 1e9,
            stage=int(msg.stage),
            command=int(msg.command),
            counter=int(msg.command_counter),
            next_command=int(msg.next_command[0]) if len(msg.next_command) else None,
            action_time_left=seconds(msg.current_action_time_remaining[0])
            if len(msg.current_action_time_remaining) else None,
            stage_time_left=seconds(msg.stage_time_left[0])
            if len(msg.stage_time_left) else None,
            designated=designated,
            blue_positive=bool(msg.blue_team_on_positive_half[0])
            if len(msg.blue_team_on_positive_half) else None))
    return records, chosen


# ------------------------------------------------------------------ leitura de log da liga

def load_records_from_log(path, proto_dir=None):
    """
    Lê um log oficial da liga (.log/.log.gz) e devolve (records, descrição), sem ROS.

    Formato: cabeçalho "SSL_LOG_FILE" + versão (int32); depois, por mensagem, em big-endian:
    timestamp (int64, ns), tipo (int32), tamanho (int32) e os bytes do protobuf. Tipo 3 = Referee.
    """
    proto_dir = Path(proto_dir) if proto_dir else PROTO_DIR
    workdir = tempfile.mkdtemp(prefix='gc_pb_')
    try:
        subprocess.run(['protoc', f'-I{proto_dir}', f'--python_out={workdir}',
                        *[str(proto_dir / f) for f in PROTO_FILES]],
                       check=True, capture_output=True, text=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        sys.exit('Falha ao gerar o código protobuf (precisa do `protoc` e de '
                 f'{proto_dir}): {getattr(exc, "stderr", "") or exc}')
    sys.path.insert(0, workdir)
    try:
        import ssl_gc_referee_message_pb2 as pb
        import ssl_vision_wrapper_tracked_pb2 as vpb
    except ImportError as exc:
        sys.exit(f'python3-protobuf ausente ou incompatível com o protoc: {exc}')

    opener = gzip.open if str(path).endswith('.gz') else open
    records, ball, t0 = [], [], None
    with opener(path, 'rb') as fh:
        if fh.read(len(LOG_MAGIC)) != LOG_MAGIC:
            sys.exit('Não parece um log SSL (cabeçalho "SSL_LOG_FILE" ausente).')
        fh.read(4)  # versão do formato
        while True:
            head = fh.read(16)
            if len(head) < 16:
                break
            stamp_ns, kind, size = struct.unpack('>qii', head)
            data = fh.read(size)
            if kind not in (LOG_REFEREE, LOG_TRACKER) or len(data) < size:
                continue
            if kind == LOG_TRACKER:
                if t0 is not None:
                    pkt = vpb.TrackerWrapperPacket()
                    pkt.ParseFromString(data)
                    if pkt.HasField('tracked_frame') and len(pkt.tracked_frame.balls):
                        tb = pkt.tracked_frame.balls[0]
                        if not tb.HasField('visibility') or tb.visibility > 0.5:
                            ball.append(((stamp_ns - t0) / 1e9, tb.pos.x * 1000.0,
                                         tb.pos.y * 1000.0))
                continue
            msg = pb.Referee()
            msg.ParseFromString(data)
            t0 = stamp_ns if t0 is None else t0
            records.append(Record(
                t=(stamp_ns - t0) / 1e9, stage=int(msg.stage), command=int(msg.command),
                counter=int(msg.command_counter),
                next_command=int(msg.next_command) if msg.HasField('next_command') else None,
                action_time_left=seconds(msg.current_action_time_remaining)
                if msg.HasField('current_action_time_remaining') else None,
                stage_time_left=seconds(msg.stage_time_left)
                if msg.HasField('stage_time_left') else None,
                designated=(msg.designated_position.x, msg.designated_position.y)
                if msg.HasField('designated_position') else None,
                blue_positive=msg.blue_team_on_positive_half
                if msg.HasField('blue_team_on_positive_half') else None))
    return records, f'log da liga: {Path(path).name}', ball


# ------------------------------------------------------------------ análise (sem ROS)

def timeline(records):
    """Só as mudanças de estágio, comando ou contador."""
    rows, prev = [], None
    for rec in records:
        key = (rec.stage, rec.command, rec.counter)
        if key != prev:
            rows.append(rec)
            prev = key
    return rows


def segments(records):
    """
    Agrupa mensagens consecutivas com o mesmo `command` e detecta anomalias.

    Anomalia = o `command` mudou mas o `command_counter` não: o GC trocou o comando sem emitir
    um comando novo. A exceção são os GOAL_* (legado): uma única mensagem seguida de HALT com o
    mesmo contador; eles vão para `blips`, e não para as anomalias.
    """
    segs, anomalies, blips = [], [], []
    prev = None
    for rec in records:
        if segs and segs[-1]['command'] == rec.command:
            seg = segs[-1]
            seg['end'] = rec.t
            seg['n'] += 1
            seg['counters'].add(rec.counter)
            if rec.action_time_left is not None:
                seg['action'].append(rec.action_time_left)
        else:
            if segs:
                segs[-1]['next'] = rec.command
                if prev is not None and rec.counter == prev.counter:
                    if prev.command in GOAL_COMMANDS:
                        blips.append((prev.t, cmd_name(prev.command), rec.counter))
                    else:
                        anomalies.append((rec.t, cmd_name(prev.command), cmd_name(rec.command),
                                          rec.counter))
            segs.append({
                'command': rec.command, 'start': rec.t, 'end': rec.t, 'n': 1,
                'counters': {rec.counter},
                'action': [] if rec.action_time_left is None else [rec.action_time_left],
                'next': None, 'stage': rec.stage})
        prev = rec
    return segs, anomalies, blips


def restart_report(segs):
    """Uma linha por reinício: duração, comando seguinte e comportamento do prazo de ação."""
    out = []
    for seg in segs:
        if seg['command'] not in RESTART_COMMANDS:
            continue
        action = seg['action']
        out.append({
            'command': cmd_name(seg['command']),
            'start': seg['start'],
            'duration': seg['end'] - seg['start'],
            'messages': seg['n'],
            'next': cmd_name(seg['next']) if seg['next'] is not None else '(fim da gravação)',
            'action_first': action[0] if action else None,
            'action_min': min(action) if action else None,
            'action_last': action[-1] if action else None,
            'counters': len(seg['counters']),
        })
    return out


def ball_report(segs, ball):
    """
    Para cada NORMAL_START/DIRECT_FREE: em quanto tempo a bola se moveu ≥ 50 mm.

    A posição de referência é a mediana da bola de 0,3 s antes a 0,1 s depois do comando. Exige 3
    amostras seguidas fora do raio de 50 mm (para ignorar ruído). Devolve uma linha por reinício.
    """
    out = []
    times = [b[0] for b in ball]
    import bisect
    for seg in segs:
        if seg['command'] not in RUNNING_COMMANDS or seg['command'] == 3:
            continue
        lo = bisect.bisect_left(times, seg['start'] - 0.3)
        hi = bisect.bisect_left(times, seg['start'] + 0.1)
        ref = ball[lo:hi]
        row = {'command': cmd_name(seg['command']), 'start': seg['start'],
               'duration': seg['end'] - seg['start'], 'timer': seg['action'][0]
               if seg['action'] else None, 'moved': None, 'ref': None}
        if len(ref) >= 5:
            rx = statistics.median(b[1] for b in ref)
            ry = statistics.median(b[2] for b in ref)
            row['ref'] = (rx, ry)
            streak = 0
            for b in ball[hi:]:
                if b[0] > seg['end'] + 0.5:
                    break
                streak = streak + 1 if math.hypot(b[1] - rx, b[2] - ry) >= BALL_MOVED_MM else 0
                if streak >= 3:
                    row['moved'] = b[0] - seg['start']
                    break
        out.append(row)
    return out


def ball_summary(rows):
    """Linhas de texto com a comparação bola-moveu × prazo."""
    measured = [r for r in rows if r['ref'] is not None]
    moved = [r for r in measured if r['moved'] is not None]
    if not measured:
        return []
    lines = [f'Bola medida em {len(measured)} de {len(rows)} reinícios; ela se moveu '
             f'≥ {BALL_MOVED_MM:.0f} mm em {len(moved)}.']
    for name in sorted({r['command'].split('_')[0] + '_' + r['command'].split('_')[1]
                        if r['command'].startswith('DIRECT') else r['command'] for r in moved}):
        grp = [r for r in moved if (r['command'].startswith('DIRECT') and name == 'DIRECT_FREE')
               or r['command'] == name]
        if not grp:
            continue
        lat = [r['moved'] for r in grp]
        before = sum(1 for r in grp if r['timer'] is not None and r['moved'] < r['timer'])
        lines.append(f'{name}: bola moveu após {min(lat):.1f}–{max(lat):.1f} s '
                     f'(mediana {statistics.median(lat):.1f} s), ANTES do prazo em {before} de '
                     f'{len(grp)} (prazo {grp[0]["timer"]:.0f} s).')
    return lines


def verdict(segs, anomalies, blips, restarts, ball_lines=()):
    """Parecer automático sobre o H-01, com as ressalvas do que a gravação permite concluir."""
    lines = []
    if not restarts:
        return ['Sem reinícios na gravação: não dá para responder ao H-01 com estes dados.']
    running = [s for s in segs if s['command'] in RUNNING_COMMANDS and s['next'] is not None]
    followers = Counter(cmd_name(s['next']) for s in running)
    lines.append(f'{len(restarts)} reinício(s)/retomada(s) analisados. Sob comandos de jogo '
                 f'(NORMAL_START, DIRECT_FREE, FORCE_START), o `command` passou depois para: '
                 + ', '.join(f'{k} ({v}×)' for k, v in followers.most_common()) + '.')
    only_stop = running and set(followers) <= {'STOP', 'HALT'}
    if anomalies:
        lines.append(f'ATENÇÃO: {len(anomalies)} mudança(s) de `command` SEM novo '
                     '`command_counter` (fora os GOAL_*): o GC alterou o comando por conta '
                     'própria. Veja a seção de anomalias; a premissa "command só muda com novo '
                     'comando" não vale.')
    elif only_stop:
        lines.append('Nenhuma mudança de `command` sem novo `command_counter` (fora os GOAL_*), e '
                     'o jogo corrido só terminou em STOP/HALT: o `command` NÃO sinaliza "jogo '
                     'corrido". Durante o Run ele permanece o do último reinício (ou '
                     'FORCE_START) até o próximo comando.')
    if blips:
        lines.append(f'{len(blips)} mensagem(ns) GOAL_* de uma só amostra, no mesmo instante do '
                     'HALT e com o mesmo contador: sinal legado de gol. Trate GOAL_* como HALT '
                     '(e não crie ramo próprio para ele).')
    crossed = [r for r in restarts if r['action_min'] is not None and r['action_min'] < 0]
    if crossed:
        lines.append(f'Em {len(crossed)} reinício(s) o `current_action_time_remaining` passou '
                     'de 0 e continuou diminuindo (valores negativos) enquanto o `command` '
                     'seguia o mesmo: o prazo do reinício esgotou e esse campo, que o Referee já '
                     'publica, indica o fim do reinício.')
    lines.extend(ball_lines)
    lines.append('Conclusão sugerida: detectar RUN localmente (bola moveu ≥ 0,05 m desde a '
                 'posição do reinício, OU `current_action_time_remaining` ≤ 0, OU FORCE_START), e '
                 'lembrar o PREPARE_* que antecedeu um NORMAL_START (o NORMAL_START não diz se é '
                 'kick-off ou pênalti).')
    return lines


def coverage(records):
    seen = {rec.command for rec in records}
    return sorted(seen), sorted(EXPECTED_COMMANDS - seen)


# ------------------------------------------------------------------ apresentação

def fmt(value, nd=1):
    return '-' if value is None else f'{value:.{nd}f}'


def report(records, topic, csv_path=None, ball=None):
    rows = timeline(records)
    segs, anomalies, blips = segments(records)
    restarts = restart_report(segs)
    seen, missing = coverage(records)

    print(f'Tópico: {topic} | {len(records)} mensagens | duração {fmt(records[-1].t, 1)} s\n')
    print('== 1. Linha do tempo (só mudanças) ==')
    print(f'{"t(s)":>8}  {"estágio":<24} {"command":<24} {"cnt":>4}  {"next":<22} {"ação(s)":>8}')
    for rec in rows:
        nxt = cmd_name(rec.next_command) if rec.next_command is not None else '-'
        print(f'{rec.t:8.1f}  {STAGES.get(rec.stage, rec.stage):<24} '
              f'{cmd_name(rec.command):<24} {rec.counter:>4}  {nxt:<22} '
              f'{fmt(rec.action_time_left):>8}')

    print('\n== 2. Reinícios e jogo corrido (H-01) ==')
    print(f'{"início":>8} {"comando":<24} {"dur(s)":>7} {"msgs":>6} '
          f'{"ação ini/mín/fim (s)":<26} seguinte')
    for r in restarts:
        act = f'{fmt(r["action_first"])}/{fmt(r["action_min"])}/{fmt(r["action_last"])}'
        print(f'{r["start"]:8.1f} {r["command"]:<24} {r["duration"]:7.1f} {r["messages"]:>6} '
              f'{act:<26} {r["next"]}')

    print('\n== 3. Transições de `command` (anterior → seguinte) ==')
    pairs = Counter((cmd_name(a['command']), cmd_name(a['next']))
                    for a in segs if a['next'] is not None)
    for (a, b), n in sorted(pairs.items(), key=lambda kv: (-kv[1], kv[0])):
        print(f'{n:>4}×  {a} → {b}')

    print('\n== 4. Anomalias (command mudou sem novo command_counter) ==')
    if anomalies:
        for t, a, b, cnt in anomalies:
            print(f'  t={t:.1f}s  {a} → {b}  (command_counter continuou {cnt})')
    else:
        print('  nenhuma')
    if blips:
        print(f'  (+ {len(blips)} mensagens GOAL_* de uma amostra, seguidas de HALT com o mesmo '
              'contador: sinal legado de gol, não é anomalia)')

    print('\n== 5. Cobertura da gravação ==')
    print('  presentes:', ', '.join(cmd_name(c) for c in seen))
    print('  ausentes :', ', '.join(cmd_name(c) for c in missing) or 'nenhum (completa)')

    ball_rows = ball_report(segs, ball) if ball else []
    if ball_rows:
        print('\n== 6. Bola em jogo (tracker da visão) ==')
        print(f'{"início":>8} {"comando":<22} {"prazo(s)":>9} {"moveu(s)":>9} {"dur(s)":>7}  '
              'bola de referência (mm)')
        for r in ball_rows:
            pos = 'sem bola' if r['ref'] is None else f'({r["ref"][0]:.0f}, {r["ref"][1]:.0f})'
            print(f'{r["start"]:8.1f} {r["command"]:<22} {fmt(r["timer"], 0):>9} '
                  f'{fmt(r["moved"], 2):>9} {r["duration"]:7.1f}  {pos}')

    print('\n== Parecer sobre o H-01 ==')
    for line in verdict(segs, anomalies, blips, restarts, ball_summary(ball_rows)):
        print(' -', line)

    if csv_path:
        with open(csv_path, 'w', newline='', encoding='utf-8') as fh:
            writer = csv.writer(fh)
            writer.writerow(['t', 'stage', 'command', 'command_counter', 'next_command',
                             'current_action_time_remaining', 'stage_time_left'])
            for rec in records:
                writer.writerow([f'{rec.t:.3f}', STAGES.get(rec.stage, rec.stage),
                                 cmd_name(rec.command), rec.counter,
                                 cmd_name(rec.next_command) if rec.next_command is not None
                                 else '', rec.action_time_left, rec.stage_time_left])
        print(f'\nCSV completo gravado em {csv_path}')


# ------------------------------------------------------------------ autoteste

def synthetic():
    """Sequência sintética: kick-off, jogo corrido, STOP, free kick, anomalia proposital."""
    recs, t, cnt = [], 0.0, 0

    def emit(command, seconds_, action=None, step=0.1, new=True):
        nonlocal t, cnt
        if new:
            cnt += 1
        n = int(seconds_ / step)
        for i in range(n):
            a = None if action is None else action - i * step
            recs.append(Record(t, 1, command, cnt, None, a, 300.0, None, True))
            t += step

    emit(0, 1)                      # HALT
    emit(1, 2)                      # STOP
    emit(4, 3)                      # PREPARE_KICKOFF_YELLOW
    emit(2, 12, action=10.0)        # NORMAL_START: prazo de 10 s esgota (vira < 0)
    emit(1, 2)                      # STOP
    emit(9, 6, action=10.0)         # DIRECT_FREE_BLUE
    emit(1, 1)                      # STOP
    emit(3, 2)                      # FORCE_START
    return recs


def selftest():
    recs = synthetic()
    segs, anomalies, blips = segments(recs)
    restarts = restart_report(segs)
    assert not anomalies and not blips, (anomalies, blips)
    assert [r['command'] for r in restarts] == [
        'PREPARE_KICKOFF_YELLOW', 'NORMAL_START', 'DIRECT_FREE_BLUE', 'FORCE_START'], restarts
    normal = restarts[1]
    assert normal['next'] == 'STOP' and abs(normal['duration'] - 11.9) < 0.2, normal
    assert normal['action_min'] < 0, normal
    seen, missing = coverage(recs)
    assert 6 in missing and 16 in missing and 2 in seen
    # anomalia: o command muda sem novo contador
    bad = list(recs)
    bad.append(Record(bad[-1].t + 0.1, 1, 1, bad[-1].counter, None, None, 300.0, None, True))
    _, anomalies, blips = segments(bad)
    assert len(anomalies) == 1 and anomalies[0][1:3] == ('FORCE_START', 'STOP'), anomalies
    # GOAL_*: 1 amostra seguida de HALT com o mesmo contador não é anomalia
    goal = [Record(0.0, 1, 2, 1, None, 5.0, 300.0, None, True),
            Record(1.0, 1, 14, 2, None, 0.0, 300.0, None, True),
            Record(1.0, 1, 0, 2, None, 0.0, 300.0, None, True)]
    _, anomalies, blips = segments(goal)
    assert not anomalies and len(blips) == 1, (anomalies, blips)
    assert len(timeline(recs)) == 8
    # bola parada até t=8.5 s e depois movendo: NORMAL_START começa em 6.0 s => ~2.5 s
    track = [(i * 0.05, 0.0, 0.0) for i in range(0, 171)]
    track += [(8.5 + i * 0.05, 100.0 * (i + 1), 0.0) for i in range(0, 200)]
    rows = ball_report(segs, track)
    normal = [r for r in rows if r['command'] == 'NORMAL_START'][0]
    assert normal['moved'] is not None and 2.4 <= normal['moved'] <= 2.8, normal
    assert any('ANTES do prazo' in line for line in ball_summary(rows)), ball_summary(rows)
    report(recs, 'sintético (autoteste)')
    print('\nAUTOTESTE OK')


def main():
    parser = argparse.ArgumentParser(description='Análise do Referee em rosbag (T03 / H-01).')
    parser.add_argument('bag', nargs='?', help='diretório (ou arquivo) da rosbag2')
    parser.add_argument('--log', help='log oficial da liga (.log ou .log.gz), lido sem ROS')
    parser.add_argument('--proto-dir', help='pasta com os .proto do GC (padrão: ssl_ros_bridge)')
    parser.add_argument('--topic', help='tópico do Referee (senão é detectado)')
    parser.add_argument('--csv', help='grava todas as mensagens neste CSV')
    parser.add_argument('--selftest', action='store_true',
                        help='roda a análise sobre dados sintéticos e verifica o resultado')
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return 0
    ball = None
    if args.log:
        records, topic, ball = load_records_from_log(args.log, args.proto_dir)
    elif args.bag:
        records, topic = load_records(args.bag, args.topic)
    else:
        parser.error('informe a bag, --log ou --selftest')
    if not records:
        sys.exit('A bag não tem mensagens do árbitro no tópico escolhido.')
    report(records, topic, args.csv, ball)
    return 0


if __name__ == '__main__':
    sys.exit(main())
