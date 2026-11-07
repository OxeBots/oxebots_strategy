"""Política estática das árvores de produção (E08, E09, E10).

Regras que não dependem de rodar a árvore e que, se quebradas, só aparecem em partida:
  - SC-24 / R-25: nenhum chute pede mais de 3,0 m/s, e nenhuma árvore de produção usa PrecisionKick
    (folha legada: o M02 já a fez passar pelo override e limitar a 3 m/s, mas o jogo usa KickBall e
    KickOnce, que confirmam o resultado e são os únicos pontos de chute das árvores novas);
  - toda guarda de ramo do mestre é ReactiveSequence (uma Sequence comum não reavalia a guarda
    enquanto o filho está RUNNING e o robô fica preso no ramo antigo; foi o bug do STOP);
  - o papel é verificado com HasRole nas árvores de atacante/defensor;
  - a árvore do defensor não usa a cor do time para decidir o lado.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

TREES = Path(__file__).resolve().parent.parent / 'behavior_trees'

# Árvores que entram no jogo. As demais (desafios técnicos, testes, legadas) ficam de fora até a A03.
PRODUCTION = [
    'master_strategy_carry.xml',
    'master_strategy_el.xml',
    'attacker_tree.xml',
    'defender_tree.xml',
    'restart_kicker.xml',
    'stop.xml',
    'free_kick.xml',
]

MAX_KICK_SPEED = 3.0


def load(name):
    return ET.parse(TREES / name).getroot()


def numeric(value):
    """Converte o atributo em float; devolve None para referências {chave} da blackboard."""
    if value.strip().startswith('{'):
        return None
    return float(value)


@pytest.mark.parametrize('name', PRODUCTION)
def test_sem_precision_kick(name):
    assert not list(load(name).iter('PrecisionKick')), (
        f'{name} usa PrecisionKick; use KickBall ou KickOnce (limitados a 3 m/s).')


@pytest.mark.parametrize('name', PRODUCTION)
def test_chutes_nao_passam_de_tres_metros_por_segundo(name):
    for tag in ('KickBall', 'KickOnce'):
        for el in load(name).iter(tag):
            speed = el.attrib.get('kick_speed')
            if speed is None:
                continue
            value = numeric(speed)
            if value is not None:
                assert value <= MAX_KICK_SPEED, (
                    f'{name}: <{tag}> kick_speed={value} acima de {MAX_KICK_SPEED} m/s (R-25).')


def test_guardas_de_ramo_do_mestre_sao_reativas():
    root = load('master_strategy_carry.xml')
    branches = [el for el in root.iter() if el.attrib.get('name', '').endswith('Branch')]
    assert branches, 'nenhum ramo "*Branch" encontrado no mestre; o teste ficou desatualizado'
    for el in branches:
        assert el.tag == 'ReactiveSequence', (
            f'ramo "{el.attrib["name"]}" é <{el.tag}>; guardas de ramo devem ser ReactiveSequence '
            '(uma Sequence comum não reavalia a guarda com o filho em RUNNING).')


def test_atacante_no_mestre_usa_hasrole_e_a_subarvore():
    root = load('master_strategy_carry.xml')
    role = next(el for el in root.iter('ReactiveSequence') if el.attrib.get('name') == 'AttackerRole')
    children = list(role)
    assert children[0].tag == 'HasRole' and children[0].attrib.get('role') == 'attacker'
    assert children[1].tag == 'SubTree' and children[1].attrib.get('ID') == 'AttackerTree'


def test_arvore_do_atacante_nao_tem_guarda_de_papel():
    # A guarda mora em quem chama; assim a árvore é só comportamento.
    root = load('attacker_tree.xml')
    assert not list(root.iter('HasRole')) and not list(root.iter('IsMyRole'))
    assert root.find("./BehaviorTree[@ID='AttackerTree']") is not None


def test_arvore_do_defensor_usa_hasrole_e_nao_depende_da_cor():
    root = load('defender_tree.xml')
    assert [el.attrib.get('role') for el in root.iter('HasRole')] == ['defender']
    assert not list(root.iter('IsMyRole'))
    for el in root.iter('IsBallInOpponentField'):
        assert 'is_yellow' not in el.attrib, (
            'IsBallInOpponentField com is_yellow: o lado deve vir de my_goal_x (troca de lados).')
        assert 'my_goal_x' in el.attrib


def test_restart_kicker_expoe_as_portas_e_termina_em_kickonce():
    root = load('restart_kicker.xml')
    tree = root.find("./BehaviorTree[@ID='RestartKickerTree']")
    assert tree is not None
    kick = list(tree.iter('KickOnce'))
    assert len(kick) == 1
    assert kick[0].attrib['target_x'] == '{target_x}' and kick[0].attrib['target_y'] == '{target_y}'
    # O IsBallClose antes do KickOnce garante o contato exigido pelo controlador.
    assert list(tree.iter('IsBallClose'))


def test_free_kick_trees_estrutura_e_papeis():
    root = load('free_kick.xml')
    our_tree = root.find("./BehaviorTree[@ID='FreeKickOurTree']")
    their_tree = root.find("./BehaviorTree[@ID='FreeKickTheirTree']")
    assert our_tree is not None, 'FreeKickOurTree ausente em free_kick.xml'
    assert their_tree is not None, 'FreeKickTheirTree ausente em free_kick.xml'

    # FreeKickOurTree usa RestartKickerTree para o kicker e freekick_support para apoio
    kicker_subtrees = [el for el in our_tree.iter('SubTree') if el.attrib.get('ID') == 'RestartKickerTree']
    assert len(kicker_subtrees) == 1
    assert kicker_subtrees[0].attrib['target_x'] == '{opp_goal_x}'
    assert kicker_subtrees[0].attrib['target_y'] == '{opp_goal_y}'

    our_slots = [el.attrib.get('slot') for el in our_tree.iter('ComputeLegalSlot')]
    assert 'freekick_support' in our_slots

    # FreeKickTheirTree usa wall para formar barreira
    their_slots = [el.attrib.get('slot') for el in their_tree.iter('ComputeLegalSlot')]
    assert 'wall' in their_slots

