# Mecanismos e controles do robô

Dois controles Xbox: o do **piloto** (porta 0) controla **somente o swerve**; o do **operador**
(porta 1) controla os subsistemas. Todos os valores de hardware (IDs CAN, relações, ganhos, ângulos)
são **placeholders** nos arquivos `*Constants.java` e precisam ser conferidos no robô.

> O código compila (com o `LimelightHelpers` real e o YAGSL 2026.1.13) e a lógica foi testada em
> simulação lógica (RPM, falhas, pivô, cascata e modos de direção). **Ainda não foi testado no robô.** Teste primeiro na simulação do WPILib (os
> mecanismos usam motores simulados com o modelo físico do Vortex) e, no robô, com o mecanismo
> livre (sem peças) e em baixa potência.
>
> **Importante:** o YAGSL do projeto é a versão **2026.1.13** (`vendordeps/yagsl-2026.1.13.json`);
> as versões a partir da 2026.8.25 (inclusive a 2026.10.03) não têm mais a API que o
> `SwerveSubsystem` usa (detalhes em `lib/README.md`, seção 11).

## Subsistema x comando
- **Subsistema** (`subsystems/...`): só o hardware — motores, sensores, leituras, log e as proteções de
  segurança (ex.: o shooter para os 3 estágios se um falhar). Expõe métodos simples (`setOutput`,
  `setTarget`, `setIndexRpm`, `indexReached`, `stopAll`...).
- **Comando** (`commands/...`): o **controle** — decide quando e como usar o subsistema:
  - `commands/collector/CollectCommand`: gira o rolo enquanto estiver ativo (LT do operador).
  - `commands/intake/MoveIntakeCommand`: pede DOWN/UP e carrega a **proteção contra clique duplo**.
  - `commands/shooter/ShootSequenceCommand`: a cascata Index → Intermediário → Shooter, as fases e
    o timeout.
  - `commands/swerve/FieldDriveCommand`, `FreedomDriveCommand` (+ `DriveModeSelector`).
  O `RobotContainer` só liga os botões a esses comandos.

## Controles

### Piloto (porta 0) — swerve
| Controle | Ação |
|---|---|
| Analógico esquerdo | Direção (frente/trás/lados) |
| Analógico direito | Giro |
| **RT** (analógico) | **Acelerar**: o robô **só anda com o RT apertado**; a velocidade sobe até 100 % com o gatilho a fundo (`kBaseSpeedFraction = 0.0`; use `0.4` para um turbo sobre 40 %) |
| **LT** | **Trocar o modo**: FIELD ⇄ FREEDOM (o controle vibra um instante) |
| START | Zera a frente (`zeroHeading()`: a direção em que o robô aponta agora vira a frente do modo FIELD) |
| X (segurar) | Trava as rodas em "X" |
| A (segurar) | GoTo de exemplo até (5 m, 4 m) desviando de obstáculos |
| B (segurar) | GoTo de exemplo até a pose (2 m, 1,5 m) virando para 180° |

- **FIELD** (`FieldDriveCommand`): a frente é **sempre** a frente que o robô tinha quando foi ligado
  (ou quando o START foi apertado), não importa para onde ele esteja virado agora. Não depende da
  aliança.
- **FREEDOM** (`FreedomDriveCommand`): **não há frente fixa**; a direção é relativa ao robô e muda
  conforme ele gira (como dirigir um carro).
- O modo atual aparece no dashboard em **"Modo de Direcao"** e no log em `Drive/Mode`.

### Operador (porta 1) — subsistemas
| Controle | Ação |
|---|---|
| **LT** (segurar) | **Coleta**: o rolo (1 Vortex) gira no sentido anti-horário; soltar para |
| **LB** | **Intake desce** (PID + perfil de movimento) |
| **RB** | **Intake sobe** |
| **RT** (segurar) | **Index → Intermediário → Shooter** em cascata; soltar para tudo |

## Coleta (`subsystems/collector`)
1 motor NEO Vortex (SPARK Flex), saída percentual (`kCollectOutput` = 80 %). Sentido padrão do
Vortex = anti-horário visto de frente para o eixo de saída; se no robô o rolo coletar para o lado
errado, inverta `kRollerInverted`.

## Intake — pivô (`subsystems/intake`)
1 motor NEO Vortex que só **desce** (DOWN) e **sobe** (UP), com **PID + perfil trapezoidal** na
roboRIO (volts), tensão máxima de 4 V e curso limitado (`kUpDegrees`..`kDownDegrees`).

**Proteção contra clique duplo:** o subsistema lembra para onde o intake está indo e **ignora**
- um pedido repetido (LB duas vezes, ou LB com o intake já abaixado), e
- qualquer pedido que chegue em menos de 0,4 s depois de um pedido aceito
  (`kRequestCooldownSeconds`).

Nesses casos nada acontece (fica registrado em `Intake/Pivot/LastRequest`). Se o pivô estiver em
falha, repetir o mesmo botão é aceito de novo (serve para tentar outra vez).

> **Ligue o robô com o intake recolhido (UP):** o encoder do motor é relativo e assume a posição UP
> ao ligar. Ajuste `kDownDegrees` um pouco **antes** do batente, para o motor não forçar contra ele.

Proteções (`SafePositionMotor`): se a tensão empurra o intake para o alvo mas ele anda para o lado
oposto → **WRONG_DIRECTION** (motor ou encoder invertido; se acontecer, mude `kPivotInverted`); se há
tensão sem progresso → **STALL_OR_NO_FEEDBACK** (travado, no batente ou encoder morto). Em ambos o
motor para e fica freado até o próximo pedido.

## Index → Intermediário → Shooter (`subsystems/shooter`)
Três motores (por padrão NEO Vortex), todos juntos no **RT do operador**, em cascata:

1. **Index** sobe (com rampa) até **5000 RPM** e não passa disso.
2. Quando o Index chega ao **"RPM x"** (`kReachedFraction` = 97 % do alvo → **4850 RPM**), o
   **Intermediário** começa e sobe até **5000 RPM**.
3. Quando o Intermediário chega ao "RPM x" (4850 RPM), o **Shooter** começa e sobe até **6000 RPM**.
4. Soltar o RT para tudo. Um estágio, depois de ligado, não desliga sozinho se o RPM cair.

Fases (`Shooter/Phase` no log e em **"Shooter"** no dashboard): `IDLE` → `SPINNING_INDEX` →
`SPINNING_INTERMEDIATE` → `SPINNING_SHOOTER` → `READY` (Shooter em 97 % de 6000 RPM). **"Shooter
Pronto"** (booleano) fica verdadeiro em `READY`.

### Lógica do encoder independente do sinal + segurança
- Todas as comparações usam o **módulo do RPM**: funciona se o encoder lê **positivo ou negativo**. O
  sentido de giro de cada estágio é só `kIndexDirection` / `kIntermediateDirection` /
  `kShooterDirection` (+1 ou −1).
- **Controle:** feedforward + P + I em **volts** (o integral compensa atrito/carga e só atua com o
  RPM estável perto do alvo; testado com perdas de 5 % a 15 %).
- **Teto:** o alvo nunca passa de 5000/5000/6000 RPM (limite no código) e sobe com rampa. Acima do
  alvo +2 % a tensão é reduzida de forma proporcional até voltar (sem cortar de uma vez). Na
  simulação o pico na subida ficou em +0,4 % (5019 / 6022 RPM).
- **OVERSPEED:** acima do alvo +8 % (em qualquer sentido) o motor para e trava.
- **INVALID_READING:** encoder com NaN/infinito/valor absurdo.
- **STALL_OR_NO_FEEDBACK:** tensão ≥ 3 V com RPM < 100 por 0,75 s (estágio travado, encoder morto ou
  cabo solto): sem isto a malha fechada empurraria a tensão máxima às cegas.
- **CONTROLLER_FAULT:** falha reportada pelo próprio SPARK.
- **Timeout:** se o Index ou o Intermediário não chegarem ao RPM em 4 s, a sequência aborta.
- **Qualquer falha para TODOS os estágios.** Só volta a funcionar soltando o RT e apertando de novo
  (isso limpa as falhas). O motivo aparece no Driver Station e no log (`Shooter/<estágio>/Fault`).
- Diagnóstico: se o sinal lido for **oposto** ao comandado, o código só avisa no Driver Station e
  registra `EncoderSignDiffersFromCommand`; continua funcionando.

## O que ajustar antes de rodar no robô
| Arquivo | Valores |
|---|---|
| `collector/CollectorConstants` | ID CAN (23), `kCollectOutput`, `kRollerInverted` |
| `intake/IntakeConstants` | ID CAN (21), relação (45), `kUpDegrees`/`kDownDegrees`, ganhos, `kPivotInverted`, tempo de espera |
| `shooter/ShooterConstants` | IDs CAN (22/24/25), relações, sentidos (±1), `kReachedFraction`, ganho `kPVoltsPerRpm`, limites de corrente |
| `Constants.DriveConstants` | `kBaseSpeedFraction`, `kRotationSpeedFraction`, zonas mortas, velocidades máximas |

IDs já usados pelo swerve (`deploy/swerve`): 1–13. Os mecanismos usam 21–25 (coleta 23, pivô 21,
Index 22, Intermediário 24, Shooter 25). `Constants.IntakeConstants` (CAN 23) é um rascunho antigo
que nenhum código usa.
