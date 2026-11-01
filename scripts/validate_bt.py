#!/usr/bin/env python3
"""
Validador estático das Behavior Trees do oxebots_strategy (backlog T02).

Confere todos os XMLs de behavior_trees/ contra as portas reais das folhas, que vêm do
executável dump_bt_model (o mesmo registro do strategy_node, em formato TreeNodesModel).

Regras (cada achado traz o código da regra):

  R0  XML inválido, <include> quebrado ou main_tree_to_execute inexistente        (erro)
  R1  ID de <BehaviorTree> duplicado entre arquivos (ou dentro de um arquivo)     (erro)
  R2  nó usado no XML que não está registrado                                     (erro)
  R3  <SubTree ID="..."> aponta para uma árvore que não existe                    (erro)
  R4  porta de entrada sem valor no XML e sem valor padrão (getInput falha)       (erro)
      (exceção: porta opcional, ou seja, descrição com "Opcional" ou listada em
       scripts/validate_bt_optional_ports.txt)
  R5  atributo do XML que não corresponde a nenhuma porta do nó                   (erro)
  R6  chave {x} lida e nunca escrita (C++, porta de saída, SetBlackboard, SubTree) (aviso)
  R7  chave {x} escrita por porta de saída e nunca lida                           (aviso)
  R8  registro do strategy_node.cpp diferente do dump_bt_model.cpp                (erro/aviso)

Uso (com o workspace compilado e carregado com 'source install/setup.bash'):

  python3 scripts/validate_bt.py                     # roda dump_bt_model sozinho
  python3 scripts/validate_bt.py --model modelo.xml  # usa um despejo já salvo
  python3 scripts/validate_bt.py --write-baseline scripts/validate_bt_baseline.txt
  python3 scripts/validate_bt.py --baseline scripts/validate_bt_baseline.txt
  python3 scripts/validate_bt.py --strict            # ignora baseline; avisos também falham

Código de saída: 0 sem achados novos; 1 com achados; 2 falha ao obter o modelo.
"""

import argparse
from collections import defaultdict
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

PACKAGE_DIR = Path(__file__).resolve().parent.parent
GENERIC_TAGS = {'Action', 'Condition', 'Control', 'Decorator'}
SKIP_TAGS = {'root', 'BehaviorTree', 'TreeNodesModel', 'include'}
SUBTREE_TAGS = {'SubTree', 'SubTreePlus'}
OPTIONAL_RE = re.compile(r'opcional|optional', re.I)
KEY_RE = re.compile(r'^\{@?([\w.]+)\}$')
BUILTIN_RE = re.compile(r'<!--\s*builtin_nodes:(.*?)-->', re.S)
REGISTER_RE = re.compile(
    r'register(?:NodeType<[^>]*>|Simple(?:Condition|Action|Decorator))\(\s*"(\w+)"')
CPP_SET_RE = re.compile(r'(?:->|\.)set(?:<[^>]*>)?\(\s*"([\w.]+)"')
CPP_GET_RE = re.compile(r'(?:->|\.)get(?:<[^>]*>)?\(\s*"([\w.]+)"')


class Finding:
    """Um achado do validador, com identificador estável para o baseline."""

    def __init__(self, rule, severity, filename, tree, detail, message):
        self.rule = rule
        self.severity = severity
        self.filename = filename
        self.tree = tree
        self.message = message
        self.ident = '|'.join([rule, filename, detail])

    def line(self, known):
        tag = 'CONHECIDO' if known else ('ERRO' if self.severity == 'erro' else 'AVISO')
        where = self.filename + (f' [{self.tree}]' if self.tree else '')
        return f'[{tag}] {self.rule} {where}: {self.message}'


def load_model(args):
    """Devolve (nós registrados {id: {categoria, portas}}, ids embutidos da biblioteca)."""
    if args.model:
        text = Path(args.model).read_text(encoding='utf-8')
    else:
        try:
            proc = subprocess.run(
                ['ros2', 'run', 'oxebots_strategy', 'dump_bt_model'],
                capture_output=True, text=True, check=True)
        except (OSError, subprocess.CalledProcessError) as exc:
            detail = getattr(exc, 'stderr', '') or str(exc)
            sys.exit('Não consegui rodar dump_bt_model (compile o pacote e faça '
                     "'source install/setup.bash', ou passe --model):\n" + detail)
        text = proc.stdout

    builtin = set()
    match = BUILTIN_RE.search(text)
    if match:
        builtin = set(match.group(1).split())
        text = BUILTIN_RE.sub('', text)

    root = ET.fromstring(text)
    model_el = root if root.tag == 'TreeNodesModel' else root.find('TreeNodesModel')
    if model_el is None:
        sys.exit('O modelo não contém <TreeNodesModel>.')

    nodes = {}
    for el in model_el:
        ports = {}
        for port in el:
            if port.tag in ('input_port', 'output_port', 'inout_port'):
                ports[port.get('name')] = {
                    'kind': port.tag.split('_')[0],
                    'default': port.get('default') or None,
                    'optional': bool(OPTIONAL_RE.search(port.text or '')),
                }
        nodes[el.get('ID')] = {'category': el.tag, 'ports': ports}

    if not any(p['default'] for n in nodes.values() for p in n['ports'].values()):
        print('AVISO: nenhuma porta do modelo tem valor padrão; a regra R4 pode gerar '
              'falsos positivos (versão da biblioteca sem "default" no modelo?).',
              file=sys.stderr)
    return nodes, builtin


def load_optional_ports(path):
    """Portas sem padrão que o nó trata como opcionais ('Nó.porta', uma por linha)."""
    optional = set()
    if path and Path(path).is_file():
        for line in Path(path).read_text(encoding='utf-8').splitlines():
            line = line.split('#', 1)[0].strip()
            if line:
                optional.add(line)
    return optional


def cpp_keys(src_dir):
    """Chaves de blackboard escritas e lidas literalmente no C++ de src/."""
    written, read = set(), set()
    for path in sorted(src_dir.glob('*.cpp')):
        text = path.read_text(encoding='utf-8', errors='replace')
        written.update(CPP_SET_RE.findall(text))
        read.update(CPP_GET_RE.findall(text))
    return written, read


def node_id(el):
    return el.get('ID') if el.tag in GENERIC_TAGS else el.tag


def validate(args, nodes, builtin):
    findings = []
    tree_files = sorted(Path(args.trees).glob('*.xml'))
    if not tree_files:
        sys.exit(f'Nenhum .xml em {args.trees}')

    # ---- 1ª passada: parse, IDs das árvores, includes e main_tree_to_execute
    docs = {}
    ids = defaultdict(list)
    for path in tree_files:
        name = path.name
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            findings.append(Finding('R0', 'erro', name, '', 'parse', f'XML inválido: {exc}'))
            continue
        docs[name] = root
        if root.get('BTCPP_format') != '4':
            findings.append(Finding('R0', 'aviso', name, '', 'formato',
                                    'root sem BTCPP_format="4"'))
        for bt in root.findall('BehaviorTree'):
            ids[bt.get('ID')].append(name)
        for inc in root.iter('include'):
            target = (path.parent / inc.get('path', '')).resolve()
            if not target.is_file():
                findings.append(Finding('R0', 'erro', name, '', f"include|{inc.get('path')}",
                                        f"<include path=\"{inc.get('path')}\"> não existe"))

    for tree_id, files in sorted(ids.items()):
        if len(files) > 1:
            for fname in sorted(set(files)):
                findings.append(Finding(
                    'R1', 'erro', fname, tree_id, f'id|{tree_id}',
                    f'ID de árvore "{tree_id}" duplicado ({len(files)} definições: '
                    f'{", ".join(sorted(set(files)))})'))

    for name, root in docs.items():
        main = root.get('main_tree_to_execute')
        if main and main not in ids:
            findings.append(Finding('R0', 'erro', name, '', f'main|{main}',
                                    f'main_tree_to_execute="{main}" não existe'))

    # ---- 2ª passada: nós, portas e chaves
    optional = load_optional_ports(args.optional_ports)
    reads, writes = defaultdict(list), defaultdict(set)
    subtree_attrs = set()

    def visit(el, fname, tree_id):
        tag = el.tag
        if tag in SKIP_TAGS:
            if tag == 'TreeNodesModel':
                return
            if tag == 'BehaviorTree':
                tree_id = el.get('ID')
            for child in el:
                visit(child, fname, tree_id)
            return
        nid = node_id(el)
        label = nid if nid else tag
        if tag in SUBTREE_TAGS:
            target = el.get('ID')
            if target not in ids:
                findings.append(Finding('R3', 'erro', fname, tree_id, f'subtree|{target}',
                                        f'<SubTree ID="{target}"> não existe'))
            for attr in el.attrib:
                if attr not in ('ID', 'name') and not attr.startswith('_'):
                    subtree_attrs.add(attr)
                    mk = KEY_RE.match(el.attrib[attr])
                    if mk:
                        reads[mk.group(1)].append((fname, 'SubTree', attr))
        elif nid not in nodes:
            findings.append(Finding('R2', 'erro', fname, tree_id, f'node|{label}',
                                    f'nó "{label}" não está registrado'))
        else:
            ports = nodes[nid]['ports']
            attrs = {k: v for k, v in el.attrib.items()
                     if k not in ('ID', 'name') and not k.startswith('_')}
            for attr, value in attrs.items():
                port = ports.get(attr)
                if port is None:
                    findings.append(Finding(
                        'R5', 'erro', fname, tree_id, f'{label}|{attr}',
                        f'<{label}> tem o atributo "{attr}", que não é uma porta do nó '
                        f'(portas: {", ".join(sorted(ports)) or "nenhuma"})'))
                    continue
                mk = KEY_RE.match(value)
                if nid == 'SetBlackboard' and attr == 'output_key':
                    writes[value].add((fname, label))
                elif mk:
                    key = mk.group(1)
                    if port['kind'] in ('input', 'inout'):
                        reads[key].append((fname, label, attr))
                    if port['kind'] in ('output', 'inout'):
                        writes[key].add((fname, label, attr))
            for pname, port in ports.items():
                if port['kind'] in ('input', 'inout') and not port['default'] \
                        and not port['optional'] and f'{nid}.{pname}' not in optional \
                        and pname not in attrs:
                    findings.append(Finding(
                        'R4', 'erro', fname, tree_id, f'{label}|{pname}',
                        f'<{label}> não recebe a porta de entrada "{pname}", que não tem '
                        'valor padrão: getInput falha e o nó devolve FAILURE'))
        for child in el:
            visit(child, fname, tree_id)

    for fname, root in docs.items():
        visit(root, fname, '')

    # ---- R6/R7: chaves da blackboard
    cpp_written, cpp_read = cpp_keys(Path(args.src))
    for key, uses in sorted(reads.items()):
        if key in writes or key in cpp_written or key in subtree_attrs:
            continue
        for fname in sorted({u[0] for u in uses}):
            findings.append(Finding(
                'R6', 'aviso', fname, '', f'key|{key}',
                f'chave {{{key}}} é lida mas nenhum C++, porta de saída ou SetBlackboard '
                'a escreve'))
    for key, uses in sorted(writes.items()):
        if key in reads or key in cpp_read:
            continue
        for use in sorted(uses, key=str):
            findings.append(Finding(
                'R7', 'aviso', use[0], '', f'out|{key}',
                f'chave {{{key}}} é escrita por <{use[1]}> e nunca lida'))

    # ---- R8: registro do strategy_node.cpp x dump_bt_model
    strategy_cpp = Path(args.strategy_node)
    if strategy_cpp.is_file():
        registered = set(REGISTER_RE.findall(strategy_cpp.read_text(encoding='utf-8')))
        in_model = set(nodes) - builtin
        for missing in sorted(registered - in_model):
            findings.append(Finding(
                'R8', 'erro', strategy_cpp.name, '', f'falta|{missing}',
                f'"{missing}" é registrado no strategy_node.cpp mas não no dump_bt_model.cpp; '
                'atualize a lista do dump'))
        for extra in sorted(in_model - registered):
            findings.append(Finding(
                'R8', 'aviso', strategy_cpp.name, '', f'sobra|{extra}',
                f'"{extra}" está no dump_bt_model.cpp mas não no strategy_node.cpp'))
    return findings


def main():
    parser = argparse.ArgumentParser(description='Validador estático das BTs (T02).')
    parser.add_argument('--trees', default=str(PACKAGE_DIR / 'behavior_trees'))
    parser.add_argument('--src', default=str(PACKAGE_DIR / 'src'))
    parser.add_argument('--strategy-node', default=str(PACKAGE_DIR / 'src' / 'strategy_node.cpp'))
    parser.add_argument('--optional-ports',
                        default=str(PACKAGE_DIR / 'scripts' / 'validate_bt_optional_ports.txt'))
    parser.add_argument('--model', help='saída salva do dump_bt_model (senão roda ros2 run)')
    parser.add_argument('--baseline', help='arquivo com achados já conhecidos (não falham)')
    parser.add_argument('--write-baseline', help='grava os achados atuais neste arquivo')
    parser.add_argument('--strict', action='store_true',
                        help='ignora o baseline e conta avisos como falha')
    args = parser.parse_args()

    nodes, builtin = load_model(args)
    findings = validate(args, nodes, builtin)

    if args.write_baseline:
        Path(args.write_baseline).write_text(
            '# achados conhecidos do validate_bt.py (um por linha)\n'
            + '\n'.join(sorted({f.ident for f in findings})) + '\n', encoding='utf-8')
        print(f'{len(findings)} achados gravados em {args.write_baseline}')
        return 0

    known = set()
    if args.baseline and not args.strict:
        known = {ln.strip() for ln in Path(args.baseline).read_text(encoding='utf-8').splitlines()
                 if ln.strip() and not ln.startswith('#')}

    order = {'erro': 0, 'aviso': 1}
    findings.sort(key=lambda f: (f.filename, order[f.severity], f.rule, f.message))
    errors = warnings = ignored = 0
    for finding in findings:
        is_known = finding.ident in known
        print(finding.line(is_known))
        if is_known:
            ignored += 1
        elif finding.severity == 'erro':
            errors += 1
        else:
            warnings += 1

    n_files = len(list(Path(args.trees).glob('*.xml')))
    print(f'\n{n_files} arquivos, {len(nodes)} nós no modelo: '
          f'{errors} erro(s), {warnings} aviso(s), {ignored} conhecido(s).')
    failed = errors > 0 or (args.strict and warnings > 0)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
