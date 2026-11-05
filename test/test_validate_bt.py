"""T02 no `colcon test`: valida os XMLs de behavior_trees/ contra as folhas registradas.

Roda o executável `dump_bt_model` (portas de todas as folhas) e passa o resultado ao
scripts/validate_bt.py. Só achados novos reprovam: os conhecidos estão em
scripts/validate_bt_baseline.txt. Para aceitar um achado de propósito, rode o validador com
--write-baseline e revise o diff do arquivo antes de commitar.
"""

import os
from pathlib import Path
import subprocess
import sys

import pytest

PACKAGE_DIR = Path(__file__).resolve().parent.parent
SCRIPT = PACKAGE_DIR / 'scripts' / 'validate_bt.py'
BASELINE = PACKAGE_DIR / 'scripts' / 'validate_bt_baseline.txt'


def test_behavior_trees_sem_achados_novos(tmp_path):
    # O CMake aponta DUMP_BT_MODEL para o executável compilado do pacote.
    dump_bt_model = os.environ.get('DUMP_BT_MODEL', '')
    if not dump_bt_model or not Path(dump_bt_model).is_file():
        pytest.fail(
            'DUMP_BT_MODEL não aponta para o executável dump_bt_model '
            f'({dump_bt_model!r}). Compile o pacote antes: ./build.sh --packages-select '
            'oxebots_strategy')

    dump = subprocess.run([dump_bt_model], capture_output=True, text=True, timeout=60)
    assert dump.returncode == 0, f'dump_bt_model falhou:\n{dump.stderr}'

    model = tmp_path / 'modelo.xml'
    model.write_text(dump.stdout, encoding='utf-8')

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--model', str(model), '--baseline', str(BASELINE)],
        capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, (
        'validate_bt.py encontrou erros novos nas árvores (veja as linhas [ERRO]):\n'
        f'{result.stdout}\n{result.stderr}')
