# Testes, validação e captura de dados da estratégia (`oxebots_strategy`)

Esta pasta e a `scripts/` reúnem as ferramentas que verificam, **sem robô e sem simulador**, se as
Behavior Trees do projeto estão bem formadas e se a biblioteca BehaviorTree.CPP (BT.CPP) se
comporta do jeito que a estratégia assume, e uma ferramenta para estudar o que o Game Controller
realmente envia. São três ferramentas independentes:

| Ferramenta | Tarefa do backlog | O que responde | Onde está |
|---|---|---|---|
| **Harness de semântica** (`test_bt_semantics`) | T01 | "A BT.CPP **instalada** faz o que o manual diz que ela faz?" | `test/test_bt_semantics.cpp` |
| **Validador estático** (`validate_bt.py` + `dump_bt_model`) | T02 | "Os XMLs de `behavior_trees/` estão **consistentes** com as folhas registradas?" | `scripts/validate_bt.py`, `src/tools/dump_bt_model.cpp` |
| **Análise do Game Controller** (`analisar_gc_bag.py`) | T03 | "Que `command` o árbitro publica durante o jogo corrido?" (hipótese H-01) | `scripts/analisar_gc_bag.py` |

Nenhuma das duas inicia ROS, publica tópicos ou mexe em robô. Rodam em segundos.

> Quando rodar: o harness, ao atualizar a BT.CPP, o ROS ou o ambiente de desenvolvimento, e antes de apostar numa
> semântica nova (uma guarda, um `Retry`, um `Parallel`). O validador, **antes de todo PR que mexa
> em XML de árvore ou em portas de folha**.

---

## Pré-requisitos

- Um ambiente com **ROS 2** (o projeto usa Jazzy) e **BehaviorTree.CPP 4.x** instalados, capaz de
  compilar o workspace. O devcontainer do repositório é uma forma de obtê-lo, mas **não é
  obrigatório**: qualquer máquina ou container equivalente serve.
- Os comandos abaixo assumem um terminal na **raiz do workspace** (a pasta que contém `src/`,
  `build/` e `install/`), com o ROS carregado (`source /opt/ros/<distro>/setup.bash`).
- O pacote precisa estar compilado: `colcon build --packages-select oxebots_strategy`.
- Para o validador: `source install/setup.bash` (ele chama
  `ros2 run oxebots_strategy dump_bt_model`).
- Python 3 sem dependências extras (só biblioteca padrão).

> **Os resultados dependem da versão instalada da BT.CPP.** O harness imprime a versão no início
> de cada execução. Os "achados" descritos neste documento foram medidos na **4.10.0** (a do
> ROS Jazzy); em outra versão o comportamento pode ser diferente, e é justamente para detectar
> isso que o harness existe.

---

## 1. Harness de semântica da BT.CPP (T01)

### O que é
Um conjunto de testes gtest que reproduz, com **folhas falsas** que contam `tick` e `halt`, as
afirmações sobre a BT.CPP em que a estratégia se apoia (Cap. 4, 5 e 23 do manual). O projeto
depende delas para segurança: por exemplo, que uma guarda reativa **interrompe** a ação em curso,
ou que um nó sem porta preenchida **falha** em vez de ler a blackboard "pelo nome".

Parte dessas afirmações foi verificada nas fontes da série 4.6, e a versão instalada pode ser outra
(a 4.10.0 no ambiente Jazzy em que foi medido). Este harness elimina a dúvida **na versão que
realmente roda** e protege contra mudanças em
atualizações futuras. Ele não usa ROS: só `behaviortree_cpp` e `gtest`.

### Como rodar
Só o harness (os testes de lint do pacote são outros e ficam de fora):

```bash
colcon test --packages-select oxebots_strategy --ctest-args -R test_bt_semantics --event-handlers console_direct+
```

Ou direto, mais rápido de repetir (depois de compilar):

```bash
./build/oxebots_strategy/test_bt_semantics
```

Para rodar um teste só (ou um grupo):

```bash
./build/oxebots_strategy/test_bt_semantics --gtest_filter='*Retry*'
```

A primeira linha de saída informa a versão testada, por exemplo
`[ harness ] BehaviorTree.CPP 4.10.0`. **Anote-a** ao reportar um resultado.

### Como ler o resultado
- `[  PASSED  ] 15 tests.` significa que todas as semânticas esperadas valem nesta versão.
- Um teste `FAILED` é um **achado**, não um defeito do teste: a biblioteca se comporta de modo
  diferente do que o projeto assumia. A mensagem de cada `EXPECT_*` diz o que era esperado.
- Se o teste falhar por **exceção** (`C++ exception with description …`), leia a mensagem: o
  harness aborta, de propósito, qualquer folha falsa chamada mais de 1000 vezes, para que um laço
  infinito da biblioteca vire falha e não trave o gtest.

### O que cada teste valida

| Teste | O que prova |
|---|---|
| `SequenceComMemoria_NaoReavaliaFilhoAnterior` | `Sequence` não reexecuta os filhos já concluídos enquanto um filho roda |
| `ReactiveSequence_ReavaliaFilhoAnteriorEmTodoTick` | `ReactiveSequence` reexecuta as guardas **a cada tick** |
| `ReactiveSequence_GuardaFalhaHaltaOFilhoEmExecucao` | guarda falha ⇒ o filho em execução recebe `halt` **uma vez**; ao voltar, recomeça (`onStart`) |
| `Fallback_ComMemoria_NaoReavaliaAlternativaAnterior` | `Fallback` simples não reavalia as alternativas anteriores |
| `ReactiveFallback_ReavaliaAlternativaAnteriorEmTodoTick` | `ReactiveFallback` reavalia as de maior prioridade a cada tick |
| `ReactiveFallback_PrioridadeMaiorPreemptaEHaltaOFilhoEmExecucao` | uma prioridade maior **preempta** e interrompe o ramo de baixa prioridade (base da Master Tree) |
| `Parallel_NaoReexecutaFilhoJaConcluido` | filhos já concluídos não são chamados de novo |
| `Parallel_HaltaOsDemaisFilhosAoDecidir` | ao decidir, o `Parallel` interrompe os filhos ainda rodando |
| `RetryInfinito_…_TickExactlyOnce_UmaTentativaPorChamada` | com `tickExactlyOnce()`, `Retry(-1)` + filho síncrono que falha = 1 tentativa por chamada |
| `RetryInfinito_…_TickOnce_RepeteNaMesmaChamadaAteOSucesso` | com `tickOnce()`, as tentativas se repetem **dentro da mesma chamada** (ver achados) |
| `GetInput_PortaSemAtributoESemPadrao_Falha_…` | porta de entrada sem atributo e sem padrão **falha**, mesmo com a chave na blackboard; com padrão `"{y}"` lê a chave |
| `StatefulAction_CicloDeVida` | `onStart` uma vez, `onRunning` repetido, **sem** `onHalted` ao concluir; novo tick recomeça |
| `StatefulAction_HaltDaArvoreChamaOnHaltedUmaUnicaVez` | `haltTree()` chama `onHalted` uma vez e é idempotente |
| `SubTreeSemAutoremap_NaoEnxergaChavesDoPai` | a blackboard da subárvore é **isolada** |
| `SubTreeComAutoremap_EnxergaChavesDoPaiEChavesNovasVazamParaORaiz` | com `_autoremap`, lê as chaves do pai **e** cria no pai as chaves novas (ver achados) |

### Achados na BT.CPP 4.10.0 (o harness já encontrou dois)
Ambos conferidos nas fontes da 4.10.0 (`decorators/retry_node.cpp`, `bt_factory.cpp`,
`blackboard.cpp`):

1. **`RetryUntilSuccessful(-1)` com filho síncrono que falha.**
   O `Retry` devolve RUNNING e emite um *wake-up*. Com `tickExactlyOnce()` isso vira uma
   tentativa por chamada. Com **`tickOnce()`**, que é o que o `strategy_node` usa
   (`strategy_node.cpp`, laço de `run()`), a árvore é **reexecutada na mesma chamada** enquanto
   houver wake-up: a chamada **não retorna até o filho ter sucesso**. Consequências a avaliar
   (hipótese, ainda não medida em jogo): enquanto um filho síncrono falha dentro de um `Retry(-1)`,
   o laço roda sem respeitar o `execution_rate`, ocupando a CPU. O `master_strategy_carry.xml` usa
   `Retry(-1)` em volta do `AttackerLogicLoop`, então vale conferir a CPU do `strategy_node` do
   atacante quando alguma guarda desse laço falha.
   *O manual diz "uma tentativa por tick (a confirmar)": está correto só para `tickExactlyOnce()`.*

2. **`_autoremap` não é "copia só as chaves que já existem no pai".**
   Com `_autoremap="true"`, a subárvore lê as chaves do pai e **cria no pai** (e, portanto, na
   raiz) as chaves novas que escreve. Só chaves privadas (começando por `_`) ficam locais.
   Isso significa que duas subárvores com `_autoremap` que escrevem a mesma chave (`dummy_ready`,
   `pkx` …) compartilham e sobrescrevem o mesmo valor.
   *O manual (Cap. 5 e o resumo de blackboard) precisa ser corrigido.*

### Como acrescentar um teste
1. Se precisar de um comportamento novo de folha, crie uma classe falsa na parte de cima do
   arquivo (siga `Scripted`, `ScriptedSync`, `IsTrue`). `Scripted` e `ScriptedSync` aceitam um
   **roteiro**: `R` = RUNNING, `S` = SUCCESS, `F` = FAILURE (um caractere por chamada; repete o
   último). O argumento `probe` dá nome ao contador.
2. Registre a classe no `SetUp()` e escreva o teste com `TEST_F(BtSemantics, Nome)`.
   `make("<corpo da árvore>", blackboard)` monta a árvore; `g_probes["nome"]` devolve os contadores
   `ticks`, `starts`, `running` e `halts`.
3. Escolha o método de tick de propósito: `tickOnce()` (como o `strategy_node`) ou
   `tickExactlyOnce()` (um tick, sem re-execução por wake-up).
4. Se o teste revelar que o manual estava errado, **não force o `EXPECT` para passar**: registre o
   comportamento real, acrescente um comentário `// Achado na <versão>: …` e corrija o manual.

---

## 2. Validador estático das árvores (T02)

### O que é
Um verificador que lê **todos** os XMLs de `behavior_trees/` (como o `strategy_node` faz ao
iniciar) e os confronta com as folhas **realmente registradas**. Ele existe porque vários defeitos
do projeto só aparecem em tempo de execução (um nó que devolve FAILURE em silêncio porque falta uma
porta). O validador os encontra antes.

São duas peças:

- **`dump_bt_model`** (C++, `src/tools/dump_bt_model.cpp`): registra as mesmas folhas do
  `strategy_node` e imprime o *TreeNodesModel* (todas as portas, tipos e valores padrão, inclusive
  os nós nativos da biblioteca). **Não instancia nenhum nó**: não precisa de robô, tópico ou ROS ativo.
- **`validate_bt.py`** (Python, `scripts/validate_bt.py`): roda o `dump_bt_model` e aplica as
  regras abaixo sobre os XMLs. Como a lógica está em Python, **ajustar uma regra não exige recompilar**.

### Como rodar

```bash
source install/setup.bash
```

```bash
python3 src/oxebots_strategy/scripts/validate_bt.py
```

Variações úteis:

| Comando | Para quê |
|---|---|
| `validate_bt.py` | relatório completo; código de saída 1 se houver **erro** |
| `validate_bt.py --model /tmp/modelo.xml` | usa um despejo já salvo (`ros2 run oxebots_strategy dump_bt_model > /tmp/modelo.xml`), sem precisar do `ros2` |
| `validate_bt.py --write-baseline scripts/validate_bt_baseline.txt` | grava os achados atuais como "conhecidos" |
| `validate_bt.py --baseline scripts/validate_bt_baseline.txt` | só os achados **novos** contam; os conhecidos aparecem como `[CONHECIDO]` |
| `validate_bt.py --strict` | ignora o baseline e faz os **avisos** também falharem (meta: zero) |
| `--trees DIR`, `--src DIR`, `--strategy-node ARQ` | caminhos alternativos |

Códigos de saída: `0` sem achados novos; `1` há erros (ou avisos, com `--strict`); `2` não foi
possível obter o modelo.

### O que valida (regras)

| Regra | Severidade | O que detecta | Por que importa |
|---|---|---|---|
| **R0** | erro | XML inválido, `<include>` quebrado, `main_tree_to_execute` inexistente | a árvore nem carrega |
| **R1** | erro | `ID` de `<BehaviorTree>` **duplicado** entre arquivos | na 4.10.0 a biblioteca **sobrescreve em silêncio**: vale a **última** definição carregada, sem erro nem aviso. O `strategy_node` carrega o diretório na ordem do sistema de arquivos e o arquivo principal por último, então qual `MainTree` vale pode mudar de máquina para máquina |
| **R2** | erro | tag usada no XML que **não está registrada** | o `strategy_node` falha ao registrar aquele arquivo (só um aviso no log) e as árvores dele ficam indisponíveis; se for o arquivo principal, a criação da árvore falha |
| **R3** | erro | `<SubTree ID="X">` apontando para uma árvore que não existe | erro ao criar a árvore que a usa |
| **R4** | erro | porta de entrada **sem atributo e sem valor padrão** | `getInput` falha e a folha devolve FAILURE em silêncio (caso `KeepDistance`/`AimAt`) |
| **R5** | erro | atributo no XML que **não é uma porta** do nó | valor ignorado ou erro na criação (caso `HasLineOfSight origin_x`) |
| **R6** | aviso | chave `{x}` **lida** que ninguém escreve (C++, porta de saída, `SetBlackboard`, atributo de `SubTree`) | a folha lê um valor que nunca existe (flags `is_corner_kick` etc.) |
| **R7** | aviso | chave `{x}` **escrita** por porta de saída e nunca lida | saída descartada: sinal de integração incompleta |
| **R8** | erro/aviso | os registros do `strategy_node.cpp` diferem do `dump_bt_model.cpp` | o validador estaria checando contra um conjunto de folhas desatualizado |

O valor de R6/R7 é aproximado: a análise é por nome de chave, global, e só vê chaves escritas
com o nome literal no C++ (`->set("chave", …)`). Trate-os como pistas, não como sentença.

### Portas opcionais
Algumas portas não têm padrão **de propósito** (a folha trata a ausência). Elas **não** devem
gerar R4. Duas formas de declará-las:

1. Escrever "Opcional" (ou "Optional") na descrição da porta no C++ (já usado em `GoToPoint`
   `face_x`/`face_y`).
2. Listar `Nó.porta` em `scripts/validate_bt_optional_ports.txt`, com um comentário do motivo
   (para portas que a folha lê de outro lugar, como `penalty_area_depth`).

### Baseline: como usar sem ficar "sempre vermelho"
O estado atual tem defeitos conhecidos (a última execução registrou **33 erros e 87 avisos** em
31 arquivos). Para o validador servir de portão em PRs sem bloquear tudo:

1. Gere o baseline uma vez: `validate_bt.py --write-baseline scripts/validate_bt_baseline.txt`.
2. Rode com `--baseline scripts/validate_bt_baseline.txt` no dia a dia: ele só falha com achados
   **novos**.
3. À medida que corrigir defeitos (Cap. 21 do manual), **regrave** o baseline para encolher.
   A meta é um baseline vazio e `--strict` verde.

O baseline identifica cada achado por `regra|arquivo|detalhe`. Renomear um arquivo ou uma porta
faz os achados aparecerem como novos: regrave quando mudar nomes de propósito.

### Como corrigir um achado
- **R4**: passe a porta no XML (`<KeepDistance robot_x="{robot_x}" robot_y="{robot_y}" …>`), dê um
  padrão `"{chave}"` à porta no C++, ou declare-a opcional (seção acima).
- **R1**: renomeie o `ID` da árvore para ser único (a duplicata nunca avisa em tempo de execução) (ou mova a árvore legada para fora de
  `behavior_trees/`). Atualize as referências `<SubTree ID>` e `main_tree_to_execute`.
- **R5**: remova o atributo ou acrescente a porta à folha.
- **R2/R3/R8**: registre o nó em `strategy_node.cpp` **e** em `src/tools/dump_bt_model.cpp`, e inclua
  o `.cpp` no `CMakeLists.txt` (nas duas listas: `strategy_node` e `BT_LEAF_SOURCES`).

### Ao criar ou alterar uma folha
Mantenha o `dump_bt_model.cpp` em dia: ele tem a sua própria lista de registros (e o
`CMakeLists.txt`, a sua lista de fontes `BT_LEAF_SOURCES`). Se esquecer, a regra R8 avisa e, para
uma folha nova, o link do `dump_bt_model` falha. O conserto de fundo é o registro único dos nós
(tarefa A05 do backlog), que eliminará essa duplicação.

---

## 3. Captura e análise do Game Controller (T03)

### Para que serve
O regulamento descreve o estado `Run` (jogo corrido), mas **não diz que valor de `command` o Game
Controller (GC) publica durante ele**. Isso importa porque o `referee_state_node` proposto
(Cap. 15 do manual) precisa decidir quando um reinício termina e o jogo corre. Se o GC mudasse o
`command` ao detectar a bola em jogo, bastaria ler o `command`; se não mudar, é preciso calcular
isso localmente (bola moveu, prazo esgotou). Esta é a **hipótese H-01**. A T03 entrega:

1. um jeito de **capturar** o tráfego do árbitro em rosbag e de **reproduzi-lo** (replay);
2. o script **`analisar_gc_bag.py`**, que lê uma bag e mostra, com evidência, o que o `command` faz
   entre um reinício e o próximo STOP/HALT;
3. a resposta à H-01 (abaixo, preliminar, e a confirmar com uma bag real).

### O que o código do Game Controller indica
Leitura do repositório `RoboCup-SSL/ssl-game-controller` (ramo `master`, consultado em
2026-10-29; a versão usada em competição pode diferir):

- O GC mantém **dois conceitos separados**: o **comando** (`Command`: HALT, STOP, NORMAL_START,
  FORCE_START, KICKOFF, PENALTY, DIRECT, TIMEOUT, BALL_PLACEMENT) e o **estado de jogo**
  (`GameState`: HALT, STOP, KICKOFF, FREE_KICK, PENALTY, **RUNNING**…). O `Referee.command`
  publicado é o **último comando**, não o estado de jogo. **O estado `RUNNING` não é publicado.**
- `NORMAL_START` **mantém o estado anterior** (kick-off ou pênalti): a mensagem não informa a qual
  reinício ele se refere. Confirma o defeito já documentado e justifica lembrar o `PREPARE_*`
  que o antecedeu.
- O estado vai a `RUNNING` por três caminhos: `FORCE_START`; o **prazo do reinício** se esgotar
  (`current_action_time_remaining` < 0: 10 s para kick-off/pênalti; 5 s na Divisão A e 10 s na B
  para free kick); ou comandos de árbitro. Não encontrei no GC nenhuma detecção de "bola
  se moveu" que mude o estado.
- `current_action_time_remaining` **é publicado** na mensagem e conta regressivamente enquanto o
  comando for `DIRECT_FREE_*`, `NORMAL_START` ou `FORCE_START`.

**Consequência provável:** o `command` **permanece o mesmo** durante o jogo corrido (o do último
reinício, ou `FORCE_START`) até o próximo comando. Logo, o `RUN` precisa ser detectado
**localmente**: bola moveu ≥ 0,05 m desde a posição do reinício, **ou**
`current_action_time_remaining` esgotado, **ou** `FORCE_START`. O cálculo local parametrizável já
previsto no manual (opção b) é o caminho correto.
*Isto vem do código; a seção seguinte confirma com um log real.*

### Resultado com um log real (confirmação)
Log analisado: `2025-05-03_06-32_UNKNOWN_MATCH_TIGERs_Mannheim-vs-TRAPS.log` (Japan Open 2025,
~20 min, 47 257 mensagens do árbitro; a divisão não está no nome do arquivo, mas o prazo de free
kick de 5 s corresponde à configuração da Divisão A do GC), baixado do servidor de logs da liga
(<https://seafile.tigers-mannheim.de/d/e85851d9bc9944bf95bb/?p=/gamelogs/2025/japan-open>, ~34 MB
comprimido) e lido direto com `--log`:

| Pergunta | Resultado (31 reinícios/retomadas) |
|---|---|
| O `command` muda quando a bola entra em jogo? | **Não.** Sob `NORMAL_START` e `DIRECT_FREE_*`, o `command` só mudou para **STOP (10×) ou HALT (10×)**; nunca para outro comando de jogo. **H-01 respondida:** o campo `command` não sinaliza o jogo corrido. |
| O GC troca o `command` sem novo `command_counter`? | **Não**, com uma exceção: 10 mensagens `GOAL_YELLOW` de **uma só amostra**, no mesmo instante de um `HALT` com o mesmo contador (sinal legado de gol). Trate `GOAL_*` como `HALT`; não crie ramo para ele. |
| O prazo do reinício é publicado? | **Sim**, em `current_action_time_remaining`: **5 s** no free kick e **10 s** no `NORMAL_START` e no `PREPARE_*`; ele **continua diminuindo e fica negativo** durante o jogo corrido (chegou a −14,9 s). Em 13 reinícios o prazo estourou antes do STOP. |
| Quando a bola **realmente** entra em jogo? (tracker da visão do mesmo log) | A bola se moveu ≥ 50 mm em **20 de 20** reinícios. **Kick-off (`NORMAL_START`):** 2,1–5,1 s depois do comando (mediana **2,2 s**), antes do prazo de 10 s em 10 de 10. **Free kick:** 4,5–6,7 s (mediana **4,7 s**), antes do prazo de 5 s em 8 de 10 (o time costuma chutar quase no limite). Ou seja, "bola movida" e "prazo esgotado" disparam no mesmo patamar no free kick, e a bola dispara bem antes no kick-off. |
| `NORMAL_START` diz se é kick-off ou pênalti? | **Não** (confirma o defeito já documentado): só o `PREPARE_*` anterior diz. |
| O `next_command` ajuda? | **Sim**, e é uma descoberta nova: em `STOP` e `BALL_PLACEMENT` ele traz **o próximo reinício** (ex.: `STOP` com `next_command = DIRECT_FREE_YELLOW`); em `PREPARE_KICKOFF_*` repete o próprio comando; em jogo corrido vem vazio. Dá para saber, ainda no STOP, quem cobra o reinício. |

### Confirmação em sete logs
Para não depender de uma só partida, a tabela reúne o log acima e **outros seis** logs oficiais de
2025 (mesmo script, mesmas regras), de eventos e divisões diferentes (três deles com times
brasileiros: ITAndroids e RoboFEI):

| Log | Reinícios | Prazo do free kick | Free kick: bola moveu após (mediana) | Antes do prazo | Kick-off: bola moveu após (mediana) | Antes do prazo |
|---|---|---|---|---|---|---|
| Japan Open, TIGERs × TRAPS | 31 | 5 s | 4,5–6,7 s (4,7) | 8 de 10 | 2,1–5,1 s (2,2) | 10 de 10 |
| German Open, SeeGoals × TIGERs | 24 | 10 s | 4,3–11,6 s (7,8) | 2 de 4 | 4,9–12,0 s (9,8) | 5 de 10 |
| Amistoso, Delft × RoboDragons | 19 | 5 s | 0,2–14,7 s (2,1) | 9 de 11 | 0,2–1,4 s (1,1) | 3 de 3 |
| RoboCup, RoboJackets × ITAndroids | 77 | 10 s | 1,0–14,6 s (3,2) | 46 de 59 | 2,6–12,2 s (5,7) | 7 de 9 |
| RoboCup, RoboJackets × A-Team | 64 | 10 s | 0,7–13,4 s (2,7) | 33 de 38 | 1,2–13,4 s (2,9) | 7 de 8 |
| RoboCup, The Bots × ITAndroids | 34 | 10 s | 1,0–4,7 s (1,8) | 21 de 21 | 1,1–4,7 s (3,3) | 5 de 5 |
| RoboCup, ITAndroids × RoboFEI (**tem pênalti**) | 70 | 10 s | 0,3–12,8 s (3,6) | 49 de 51 | 0,7–2,5 s (1,3) | 7 de 7 |
| **Total** | **319** | | | **168 de 194 (87%)** | | **44 de 52 (85%)** |

O que os seis logs mostram em conjunto:
- **O `command` nunca sinaliza o jogo corrido.** Em todos, sob `NORMAL_START`, `DIRECT_FREE_*` e
  `FORCE_START`, ele só passa para `STOP` ou `HALT`.
- **A bola se move sempre antes do prazo? Não.** Em cerca de **13–15%** dos reinícios a bola só se
  moveu **depois** de o prazo esgotar (até 14,7 s), e em alguns nem se moveu. Por isso a regra
  precisa dos **dois** critérios: usar só o prazo atrasaria o `RUN` na maioria (a bola costuma se
  mover bem antes, mediana de 2 a 8 s), e usar só a bola deixaria o `RUN` preso quando o time demora.
- **O prazo do free kick depende do evento:** 5 s (Japan Open e amistoso) ou 10 s (os demais). Leia
  o valor de `current_action_time_remaining` no início do reinício, em vez de fixar um número no
  código.
- **`FORCE_START` e `TIMEOUT_YELLOW/BLUE` aparecem** nesses logs e se comportam como os demais
  comandos.
- **Inconsistências de 1 amostra (3 em 319 reinícios):** uma mensagem com o `command_counter`
  **antigo** e um `command` diferente, ~1 ms antes do comando novo. Exemplos: `STOP` com o contador
  921 antes do `HALT 922` (em `RoboJackets × ITAndroids`, com o `stage_time_left` voltando 16 s) e
  `DIRECT_FREE_YELLOW` com os contadores 427 e 512 antes do `HALT 428`/`HALT 513` (em
  `ITAndroids × RoboFEI`). Parece um estado intermediário ou revertido publicado antes do comando
  seguinte (a causa exata não foi determinada). O efeito é o mesmo: um consumidor que reage ao
  `command` isolado vê um comando falso por 1 ms. Um deduplicador que só aceita um comando novo
  quando o `command_counter` **aumenta** ignora essas amostras.

### O pênalti (confirmado em dados reais)
O log `ITAndroids × RoboFEI` (Divisão B) tem um pênalti completo. Sequência observada:

| t (s) | Mensagem | O que mostra |
|---|---|---|
| 2020,1 | `STOP` 535, `next_command = PREPARE_PENALTY_YELLOW` | ainda no STOP, o árbitro já informa **o pênalti que vem** e quem cobra |
| 2025,2 | `PREPARE_PENALTY_YELLOW` 536, prazo 10 s | `next_command` repete o próprio comando |
| 2027,5 | `NORMAL_START` 537, prazo volta a 10 s | **idêntico ao `NORMAL_START` do kick-off**: só o `PREPARE_PENALTY_*` anterior diz que é pênalti |
| 2028,2 | a bola se move (0,73 s depois) | o chute |
| 2037,46 | prazo chega a 0: `game_events` ganha **`PENALTY_KICK_FAILED`**, `next_command = DIRECT_FREE_BLUE` | o GC declara a cobrança falha |
| 2037,46 | `STOP` 538, 2 ms depois | o jogo **para** sozinho |

Duas consequências para o projeto:
1. **No pênalti, "prazo esgotado" NÃO significa `RUN`.** O estado do GC é `PENALTY` (não `RUNNING`) e,
   ao zerar o prazo, o código (`process_penalty.go`) gera o evento `PENALTY_KICK_FAILED` ("Time run
   out") e o jogo vai a `STOP` com um free kick para o time que defendia. O `RUN` do kick-off e do
   free kick não vale aqui: o pênalti termina em gol ou em `STOP`.
2. O `NORMAL_START` do pênalti dura **até 10 s**, e o goleiro adversário deve continuar na linha de
   gol e os demais robôs atrás da bola durante todo esse tempo.

**O que os logs não cobrem:** `PREPARE_PENALTY_BLUE` (só apareceu o amarelo; é simétrico), a fase de
**disputa de pênaltis** (`PENALTY_SHOOTOUT`) e a **Entry Level**. Os prazos (5 s ou 10 s no free kick)
mudam por evento: leia-os do `current_action_time_remaining`.

**Decisão sugerida para o `referee_state_node`** (baseada nos sete logs):
- **Kick-off e free kick:** `RUN` = `FORCE_START`, **ou** `current_action_time_remaining` ≤ 0,
  **ou** bola movida ≥ 0,05 m desde a posição do reinício (o que vier primeiro). Os dois
  critérios se complementam: a bola se move antes do prazo em ~85% dos casos, mas nos ~15%
  restantes só o prazo encerra o reinício.
- **Pênalti:** não use as regras acima. O pênalti termina em gol ou em `STOP` (com
  `PENALTY_KICK_FAILED` em `game_events` quando o prazo esgota).
- Guardar o último `PREPARE_*` para interpretar o `NORMAL_START`; tratar `GOAL_*` como `HALT`;
  usar `next_command` durante o `STOP` para preparar o reinício.
- Aceitar um comando novo **só quando o `command_counter` aumenta** (ignora as amostras
  espúrias de 1 ms).

### O que o `.proto` do Game Controller documenta (e o projeto ainda não usa)
O arquivo `ssl_gc_referee_message.proto` (em `ssl_ros_bridge/ssl_league_protobufs/proto`) define o
formato e **documenta a semântica de cada campo**. Pontos relevantes, conferidos também no log real:

| Campo / comando | O que o `.proto` diz | Para que serve no projeto |
|---|---|---|
| `GOAL_YELLOW/BLUE` (14/15) | "Deprecated: use o campo `score` das `TeamInfo`", só informativo | confirma o achado: trate como `HALT` e, se precisar do gol, leia `score`. No log: 10 sinais `GOAL_YELLOW` = placar final 10×0 |
| `TeamInfo.goalkeeper` | número do goleiro **de cada time**, publicado pelo árbitro | resolve o goleiro fixo em `robot_id == 0` (tarefa K01). No log: amarelo 0, azul 0 e depois 2 (muda durante o jogo) |
| `TeamInfo.max_allowed_bots` | máximo de robôs em campo (por divisão e cartões) | degradação por cartão amarelo/vermelho (tarefa P03). No log: 8 (Divisão A) |
| `yellow_card_times`, `red_cards`, `yellow_cards` | cartões e tempo restante | idem |
| `TeamInfo.can_place_ball`, `ball_placement_failures` | se o time pode/ainda pode fazer o ball placement | decidir `BallPlacementOur` ou o modo desabilitado (tarefa E04) |
| `designated_position` | em **mm, coordenadas da visão**; só existe no ball placement | alvo do `PlaceBall` e do corredor a evitar |
| `blue_team_on_positive_half` | de que lado o time azul defende | `side_sign`/`own_goal_x` (tarefas D03/D04). No log: `False` o tempo todo |
| `next_command` | "o comando que virá depois da parada e do ball placement" | saber quem cobra o próximo reinício ainda no STOP |
| `current_action_time_remaining` | "não é reiniciado; pode ficar negativo"; vale para free kick, kick-off, pênalti, force start e ball placement | prazo do reinício (já tratado acima) |
| `game_events` | eventos detectados "desde o último estado RUNNING"; **limpos quando o jogo continua** | o código do GC os zera ao emitir `NORMAL_START`/`DIRECT`/`FORCE_START` (e também `next_command` e a posição de placement), por isso `next_command` vem vazio em jogo corrido |
| `command_timestamp` | muda **só quando um comando novo é emitido** | distinguir comando novo de repetição (complementa `command_counter`) |

Tipos de `game_events` presentes no log: `GOAL`, `POSSIBLE_GOAL`, `BALL_LEFT_FIELD_*`,
`ATTACKER_DOUBLE_TOUCHED_BALL`, `BOT_SUBSTITUTION`, `PLACEMENT_SUCCEEDED/FAILED`,
`BOT_INTERFERED_PLACEMENT`. A lista completa está em `ssl_gc_game_event.proto`.

### Passo 1: obter uma bag do árbitro
**Opção A. Log oficial de partida da liga** (tem o árbitro real, com os comandos de um jogo):
1. Escolha um log em <https://ssl.robocup.org/collected-data/> (links para os repositórios de
   logs). São arquivos grandes (`.log.gz`; o menor de 2025 em Japan Open tem ~34 MB).
2. **Para só analisar, não precisa de ROS nem de bag:** o script lê o `.log`/`.log.gz` direto
   (`--log`, veja o Passo 2). Precisa de `python3-protobuf` e do `protoc`.
3. **Para reproduzir (replay)** é preciso uma bag. Descomprima (o `log2bag` não lê `.gz`):
   `gunzip -k jogo.log.gz`; converta: `ros2 run ssl_ros_bridge log2bag jogo.log`. A bag sai ao lado
   do log, com os tópicos `/referee_messages` e `/vision_messages`.

**Opção B. Gravar você mesmo**, com o Game Controller e o simulador rodando (cobre exatamente os
comandos que interessam e a Divisão do time):
```bash
ros2 bag record -o gc_todos_comandos /gc_multicast_bridge/referee_messages /ssl_vision_bridge/vision_messages
```
Com a gravação ligada, dispare pela interface do GC (normalmente em `http://localhost:8081`),
nesta ordem, **para o time amarelo e para o azul**: HALT, STOP, FORCE_START; kick-off
(PREPARE_KICKOFF → NORMAL_START) e **deixe a bola correr**; free kick (DIRECT_FREE) e deixe a bola
correr, ou deixe o prazo estourar; pênalti (PREPARE_PENALTY → NORMAL_START); BALL_PLACEMENT;
TIMEOUT; e um STOP depois de cada trecho de jogo corrido.

### Passo 2: analisar
```bash
source install/setup.bash
```

```bash
python3 src/oxebots_strategy/scripts/analisar_gc_bag.py caminho/da/bag
```

Direto de um log da liga, **sem ROS e sem bag** (descomprime `.gz` sozinho):

```bash
python3 src/oxebots_strategy/scripts/analisar_gc_bag.py --log jogo.log.gz
```

O relatório tem cinco seções (seis com `--log`): **(1)** linha do tempo só com as mudanças; **(2)** cada reinício,
com a duração em que o `command` permaneceu, o comando seguinte e o comportamento do
`current_action_time_remaining` (início / mínimo / fim); **(3)** tabela de transições
`comando → comando`; **(4)** **anomalias**, isto é, `command` que mudou **sem** novo
`command_counter` (se houver, o GC muda o comando sozinho e a hipótese acima cai); **(5)** cobertura:
quais comandos aparecem e **quais faltam** (se faltar algo, regrave); **(6)** bola em jogo, só com `--log`. Termina com um **parecer**
automático sobre a H-01.

Com `--log`, o script lê também o **tracker da visão** do mesmo arquivo (bola em mm) e acrescenta a
seção 6: para cada `NORMAL_START`/`DIRECT_FREE`, em quanto tempo a bola se moveu ≥ 50 mm contra o
prazo do reinício. (Na rosbag essa seção não aparece: ela não lê o tópico de visão.)

Outras opções: `--topic T` (força o tópico), `--proto-dir DIR` (onde estão os `.proto` do GC; o
padrão é o do `ssl_ros_bridge`), `--csv saida.csv` (todas as mensagens, para planilha) e
`--selftest` (roda a lógica sobre dados sintéticos, sem ROS e sem bag).

### Passo 3: usar a bag como replay
Reproduz o árbitro gravado como se fosse ao vivo, para testar o `strategy_node` (e, no futuro, o
`referee_state_node`) sem Game Controller:
```bash
ros2 bag play caminho/da/bag --remap /referee_messages:=/gc_multicast_bridge/referee_messages /vision_messages:=/ssl_vision_bridge/vision_messages
```
(Se a bag veio da Opção B, os tópicos já têm os nomes ao vivo e o `--remap` não é necessário.)

### O que fazer com a resposta
Registre o resultado no Cap. 15 do manual (tabela "comando observado em `Run`") e fixe o padrão do
`referee_state_node`. **Guarde as bags fora do Git** (são grandes): em um armazenamento do time, com
o caminho anotado no manual.

### Limitações
- A lógica da análise foi testada com dados sintéticos (`--selftest`) e com **sete logs reais**
  (`--log`, 319 reinícios). A leitura de uma **rosbag** (`rosbag2_py`, sem `--log`) usa a mesma análise, mas **ainda
  não foi executada**; se falhar ao abrir uma bag, reporte o erro com a versão do ROS.
- Os logs são de partidas das Divisões A e B de 2025 (não há log da Entry Level). Só um tem pênalti
  (cobrador amarelo); não há `PREPARE_PENALTY_BLUE` nem disputa de pênaltis (`PENALTY_SHOOTOUT`). Os
  prazos (5 s ou 10 s no free kick) podem diferir na Entry Level.
- O resultado vale para a versão do Game Controller usada naquele evento (2025).

---

## Arquivos gerados e versionamento
- Os resultados dos testes (XML do gtest, logs do CTest) são escritos em `build/` e `log/`, que já
  estão no `.gitignore` do workspace e do pacote. Nada precisa ser acrescentado.
- O validador **não grava nada** por conta própria (só imprime), a menos que você use
  `--write-baseline`. Para despejos temporários, use `/tmp`.
- As **rosbags** do árbitro (T03) são grandes: **não** vão para o Git. Guarde-as em armazenamento
  do time e anote o caminho no manual.
- **Devem ser versionados** (são estado compartilhado do time): `scripts/validate_bt_baseline.txt`
  (achados conhecidos) e `scripts/validate_bt_optional_ports.txt` (portas opcionais).

## Limitações atuais
- O validador **não executa** as árvores: não prova que um ramo é alcançado, só que o XML está
  consistente. Os testes de comportamento (Master Tree com blackboard mock, cenários SC-xx) são as
  tarefas T04 a T06.
- Não verifica **tipos** das chaves da blackboard entre produtor e consumidor.
- R6/R7 são heurísticos (ver acima).
- O harness usa folhas falsas: valida a **biblioteca**, não as folhas reais do projeto (T04).
- O validador ainda não roda sozinho em CI; chame-o antes de abrir PR.

## Se algo não funcionar
| Sintoma | Provável causa / o que fazer |
|---|---|
| `Não consegui rodar dump_bt_model` | pacote não compilado ou `source install/setup.bash` esquecido; ou use `--model` |
| `nenhuma porta do modelo tem valor padrão` | a versão da biblioteca não grava `default` no modelo: R4 ficaria cheio de falsos positivos; reporte |
| link do `dump_bt_model` falha após criar uma folha | a fonte nova não está em `BT_LEAF_SOURCES` no `CMakeLists.txt` |
| R8 "registrado no strategy_node.cpp mas não no dump" | acrescente o registro em `src/tools/dump_bt_model.cpp` |
| teste do harness aborta com "chamada mais de 1000 vezes" | uma folha falsa foi chamada em laço infinito pela biblioteca: é um achado (veja `Retry`) |
| `colcon test` mostra muitas falhas de `uncrustify`, `cppcheck`, `flake8`… | são os linters do pacote (`ament_lint_auto`), anteriores a estes testes; filtre com `--ctest-args -R test_bt_semantics` |

## Referências
- Manual da Estratégia OxeBots, **Cap. 23 (Testes e validação)**: pirâmide de testes, validador
  estático e teste de semântica (`docs/manual`, na branch `feat/documentacao`).
- Backlog: tarefas T01 e T02 em `docs/manual/anexos/backlog.csv`.
- Documentação da biblioteca: <https://www.behaviortree.dev/> e
  <https://github.com/BehaviorTree/BehaviorTree.CPP> (tag `4.10.0`).
