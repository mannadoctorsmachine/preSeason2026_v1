# Biblioteca do robô (`frc.robot.lib`) — guia completo

Este documento explica **tudo que existe na lib, como cada peça funciona e como ela se conecta ao
resto do código** (subsistemas, `RobotState`, `RobotContainer`, PathPlanner e AdvantageKit).

> **Status:** o projeto inteiro **compila sem erros** (javac 21, `--release 17`) contra WPILib
> 2026.1.1, REVLib 2026.0.5, Phoenix 6 26.3.0, AdvantageKit 26.0.2, PathPlanner 2026.1.2,
> Studica 2026.0.0, o `LimelightHelpers.java` real e **YAGSL 2026.1.13** (ver seção 11). A lógica nova de RPM, posição, pivô,
> cascata do shooter e modos de direção foi testada em simulação lógica (95 verificações). Ainda
> **não** foi testado no robô nem na simulação completa do WPILib (precisa das bibliotecas nativas).

---

## 1. Visão geral e regras de ouro

```
                       ┌────────────────────────── lib ──────────────────────────┐
  hardware real        │                                                          │
  ────────────         │  motors/      sensors/       vision/      navigation/    │
  TalonFX / Spark      │  MotorController  Gyro         VisionCamera  ObstacleField│
  Pigeon / NavX        │  (interface)      AbsoluteEnc. VisionManager LiangBarsky  │
  CANcoder / CANrange  │  + Inputs         DigitalSensor              GoToPoseCmd  │
  Limelight            │  + Factory        DistanceSens.              AllianceUtil │
                       └──────────────────────────────────────────────────────────┘
                                      ▲ usados por ▲
        subsystems/ (swerve, vision, intake, shooter, elevator, arm …)  +  RobotState
                                      ▲ montados em ▲
                                RobotContainer  ──►  Robot (LoggedRobot / AdvantageKit)
```

1. **Desacoplamento de subsistemas.** `SwerveSubsystem` só cuida de cinemática e motores;
   `VisionSubsystem` só cuida das câmeras. Os dois **não se conhecem**: ambos alimentam o
   `RobotState`, que funde tudo e é a única fonte da pose do robô.
2. **Abstração de hardware (camada IO).** O código de cima só conhece interfaces
   (`MotorController`, `Gyro`, `AbsoluteEncoder`, `DigitalSensor`, `DistanceSensor`,
   `VisionCamera`). Trocar Kraken por NEO, Pigeon por NavX ou Limelight por outra câmera é mexer
   **em uma classe de baixo nível ou numa constante**, nunca nos subsistemas.
3. **Padrão idêntico em todos os pacotes:**

   | Peça | Papel |
   |---|---|
   | Interface | O que o resto do código enxerga |
   | Classe `*Inputs` | Os dados lidos do hardware (implementa `LoggableInputs` → log + replay) |
   | Implementação real | Fala com o fabricante (Phoenix 6, REV, Studica, Limelight) |
   | Implementação simulada | Roda no computador sem hardware |
   | Fábrica | Escolhe real ou simulada (`MotorFactory`, `SensorFactory`) |

4. **Referencial único.** Tudo trabalha no referencial do **campo azul** (origem no canto azul,
   +X em direção ao lado vermelho, +Y para a esquerda, ângulos positivos anti-horários). A
   conversão para a aliança vermelha é feita só nas bordas (joystick, destinos, obstáculos,
   trajetórias).
5. **Unidades.** Metros, segundos, radianos, **rotações** para motores/encoders (já no eixo de saída
   do mecanismo, considerando a relação de transmissão).

---

## 2. `lib/motors` — motores

Já existia antes das outras partes (as classes abaixo foram lidas, não reescritas; a única adição
feita depois foi `MotorFactory.createOrSim`).

| Classe | Função |
|---|---|
| `MotorController` | Interface unificada: `setPercentOutput`, `setVoltage`, `setPosition`, `setVelocity`, `setMotionProfilePosition` (Motion Magic/MAXMotion), versões com **feedforward arbitrário**, `stop`, `setBrakeMode`, `setInverted`, `setEncoderPosition`, leituras (`getPosition`, `getVelocity`, corrente, temperatura), `follow`, `updateInputs`, `applyConfiguration`. Tudo em unidades do **mecanismo** (rotações, rot/s). Há versões em radianos (`default`). |
| `MotorConfig` | Configuração completa: modo neutro, inversão, limites de corrente (estator/supply), rampas, `gearRatio`, `PIDConfig`, `MotionProfileConfig`, soft limits, continuous wrap. Tem métodos `withXxx` encadeáveis. |
| `PIDConfig` | kP/kI/kD + feedforward kS/kV/kA/kG, iZone, saída máxima, tolerância. |
| `MotionProfileConfig` | Velocidade de cruzeiro, aceleração e jerk (em rotações de saída). |
| `MotorType` | `TALON_FX`, `SPARK_MAX`, `SPARK_FLEX`, `SIMULATED`. |
| `TalonFXMotor` | Implementação Phoenix 6. Reaproveita *requests* de controle (sem alocação no loop), lê sinais com `BaseStatusSignal.refreshAll`, aplica a configuração em um único `TalonFXConfiguration`. Inversão é feita por `InvertedValue` (não existe mais `setInverted` de runtime nos moldes antigos). A relação de transmissão vai em `Feedback.SensorToMechanismRatio`. |
| `SparkMotor` | Implementação REV (NEO/NEO550/Vortex). |
| `SimulatedMotor` | Modelo DC simplificado (`DCMotor`), com PID interno e dinâmica de 1ª ordem. |
| `MotorFactory` | `create(...)`, `createTalonFX`, `createSparkMax`, `createSparkFlex`, `createSimulated` e **`createOrSim(canId, bus, tipoReal, config)`** (adicionado depois): cria o motor real e **troca automaticamente por simulado em simulação**, já com o modelo físico do motor real (Vortex = `DCMotor.getNeoVortex`, NEO = `getNEO`, Kraken = `getKrakenX60`). É o método recomendado para subsistemas. |
| `MotorInputs` | Telemetria do motor. **Agora implementa `LoggableInputs`** (ver seção 7). |
| `SafeVelocityMotor` | **Controle de RPM com proteções** (seção 2.1). |
| `SafePositionMotor` | **Controle de posição com proteções** (seção 2.2). |
| `MotorFault` | Enum das falhas detectadas pelos dois controladores acima. |

### 2.1 `SafeVelocityMotor` — RPM com segurança
Malha fechada **na roboRIO**, em **volts**: `V = kS + kV·alvo + kP·(alvo − RPM) + kI·∫erro`,
aplicada com `motor.setVoltage(direção · V)`. O integral compensa atrito/carga e tem anti-windup
(não acumula durante a rampa, com a saída saturada, nem com o RPM ainda subindo). Funciona com qualquer `MotorController` (Vortex, NEO, Kraken ou
simulado) e **se comporta igual no robô e na simulação** (as unidades de ganho dos PIDs internos
dos controladores variam por fabricante/firmware; volts não).

- **Lógica independente do sinal do encoder:** todas as comparações usam o **módulo** do RPM
  (`getSpeedRpm()`, `hasReached(fração)`). A realimentação é sempre negativa, então o encoder pode
  ler positivo ou negativo. O sentido de giro vem só de `Config.direction` (+1/−1 = sinal da
  tensão). Se o sinal lido for oposto ao comandado, o código só **avisa** e continua funcionando.
- **Alvo:** sempre limitado a `maxRpm` e com **rampa** de subida (`rampRpmPerSec`).
- **Teto suave:** acima de `maxRpm +2 %` a tensão é reduzida de forma proporcional (e o integral é
  esquecido) até voltar ao limite, sem cortar/frear de uma vez (isso causava oscilação).
- **Falhas (travam o motor até `clearFault()`):** `OVERSPEED` (> `maxRpm +8 %`, nos dois
  sentidos), `INVALID_READING` (NaN/infinito/absurdo), `STALL_OR_NO_FEEDBACK` (tensão ≥ 3 V e
  RPM < 100 por 0,75 s: travado, encoder morto ou cabo solto) e `CONTROLLER_FAULT`.
- Desabilitado: motor parado e controle zerado. Tudo vai para o AdvantageKit
  (`<nome>/SpeedRpm`, `SignedRpm`, `RampedTargetRpm`, `OutputVolts`, `Fault`...).
- Ganhos em **V por RPM**: `kVFromFreeSpeed(rpmLivresNoEixo)` dá o feedforward teórico
  (Vortex 6784 RPM → 0,00177 V/RPM).

### 2.2 `SafePositionMotor` — posição com segurança
Perfil trapezoidal + PID + feedforward (`kS`, `kV`, gravidade `kG·cos`) em **volts**, curso
limitado por software (`min`/`max`) e tensão máxima (`maxVolts`). Como a malha está no código, um
encoder com o sinal trocado viraria realimentação positiva (o mecanismo dispararia); por isso há
um **guarda**: se a tensão empurra o mecanismo para o alvo mas ele anda para o lado oposto, o motor
para na hora (`WRONG_DIRECTION`); tensão sem progresso vira `STALL_OR_NO_FEEDBACK` (travado/batente/
encoder morto); posição NaN vira `INVALID_READING`. Desabilitado, o perfil acompanha a posição real
(sem pulo ao habilitar). `resetPosition(rot)` informa a posição real conhecida (ex.: ligar o robô
com o mecanismo recolhido).

### Como o resto do código usa
```java
MotorController m = MotorFactory.createOrSim(id, "", MotorType.TALON_FX,
    new MotorConfig().withGearRatio(6.12).withPID(0.3, 0, 0, 0.1, 0.73, 0));
...
// todo ciclo, no periodic():
m.updateInputs(inputs);                       // lê o hardware (1x por ciclo!)
Logger.processInputs("Swerve/FL/Drive", inputs); // loga / faz replay
m.setVelocity(rps);                           // comanda
```

> ⚠️ **Chame `updateInputs` exatamente uma vez por ciclo por motor.** No `SimulatedMotor` essa
> chamada também avança a física (dt = 0,02 s); chamar duas vezes acelera a simulação.

---

## 3. `lib/sensors` — sensores

Escolhas de hardware e motivo:

| Categoria | Hardware | Uso típico |
|---|---|---|
| Giroscópio | **Pigeon 2.0** (principal), NavX2 (alternativa) | Orientação do robô (odometria) |
| Encoder absoluto | **CANcoder**, REV Through Bore, Thrifty Absolute | Zerar o esterço do swerve, braços, pivôs |
| Digital | Beam break, limit switch (DIO) | Detecção de peça, fim de curso |
| Distância | **CANrange** (time-of-flight) | Detecção de peça sem fiação DIO |

### 3.1 Gyro
- `Gyro` (interface): `getYaw/Pitch/Roll` (`Rotation2d`), `getYawVelocityRadPerSec`, `setYaw`,
  `zeroYaw`, `isConnected`, `updateInputs`. Yaw **contínuo**, CCW positivo.
- `Pigeon2Gyro`: Phoenix 6; sinais de yaw e velocidade a 100 Hz, pitch/roll a 50 Hz; `connected`
  vem do `StatusCode` do `refreshAll`.
- `NavXGyro`: Studica `AHRS`; **inverte o sinal** (NavX é horário-positivo) e implementa `setYaw`
  por offset em software.
- `SimulatedGyro`: o `SwerveSubsystem` injeta a velocidade angular (`setYawVelocityRadPerSec`) e o
  yaw é integrado a cada `updateInputs`.

### 3.2 AbsoluteEncoder
- `AbsoluteEncoderConfig`: `offsetRotations` (leitura do ímã com o mecanismo no zero), `inverted`,
  `minRotations` (faixa `[-0.5, 0.5)` ou `[0, 1)`), e `wrap()`.
- `CANcoderEncoder`: offset/direção/descontinuidade aplicados **no hardware** (`MagnetSensor`).
- `DutyCycleAbsoluteEncoder`: Through Bore/Thrifty em DIO; offset, inversão e wrap em software;
  velocidade por diferença entre leituras tratando a virada 1,0→0,0.
- `SimulatedAbsoluteEncoder`: posição definida por `setSimulatedPosition`.

### 3.3 DigitalSensor
- Trabalha com o conceito de **"ativado"** (`isTriggered`), independente de a chave ser NA ou NF.
- `DigitalInputSensor`: `DigitalInput` + `inverted` + `Debouncer` opcional.
- `asTrigger()` devolve um `Trigger` do command-based (use em bindings e command groups).
- `SimulatedDigitalSensor`: `setTriggered(boolean)`.

### 3.4 DistanceSensor
- `CANrangeSensor`: limiar de detecção, histerese e FOV configurados **no hardware**
  (`ProximityParams`/`FovParams`); lê distância, `isDetected` e intensidade de sinal.
- `DistanceSensorConfig`: `detectionThresholdMeters`, `hysteresisMeters`, `fovDegrees`.
- `SimulatedDistanceSensor`: `setSimulatedDistance`, com histerese também.

### 3.5 SensorFactory
`createGyro`, `createAbsoluteEncoder`, `createDigital`, `createDistance`. Em **simulação devolve
sempre a versão simulada**. Enums: `GyroType`, `AbsoluteEncoderType`, `DistanceSensorType`.

---

## 4. `lib/vision` — Limelights com prioridade

### 4.1 Isolamento do `LimelightHelpers`
Somente `LimelightCamera` importa `frc.robot.LimelightHelpers`. O resto só conhece `VisionCamera`.
> O arquivo oficial `LimelightHelpers.java` (não é vendordep) fica em `src/main/java/frc/robot/`.
> As assinaturas que o `LimelightCamera` usa foram conferidas contra ele. Ele depende de
> `edu.wpi.first.net.PortForwarder` (módulo `wpinet`, que já vem com o WPILib).

### 4.2 Peças
| Classe | Função |
|---|---|
| `VisionCamera` | Interface: `getConfig`, `setRobotOrientation(yaw, yawRate)`, `updateInputs`, `setTagFilter`, `setPipeline`. |
| `LimelightCamera` | Implementação real. `connected` vem do **heartbeat** (`hb`) no NetworkTables; descarta amostras repetidas (mesmo `timestamp`); usa MegaTag 2 ou 1 conforme a config; sempre referencial **wpiBlue**. |
| `SimulatedVisionCamera` | Não vê nada por padrão; `setSimulatedMeasurement(...)` injeta leitura (testes de fallback). |
| `VisionCameraConfig` | `name` (hostname da Limelight), `priority`, `useMegaTag2`, `maxAvgTagDistanceMeters`, `minTagCount`, `xyStdDevBase`, `thetaStdDevRad`, `stdDevScale`, `enabled`. |
| `VisionInputs` | Telemetria bruta (loggável): conectada, pose, timestamp, nº de tags, distância média. |
| `VisionMeasurement` | Medição **já filtrada** (pose, timestamp, `stdDevs` `[x, y, θ]`, câmera, prioridade). |
| `VisionManager` | A camada intermediária (abaixo). |

### 4.3 `VisionManager` — o que faz a cada ciclo
`update(robotYaw, yawRate)`:
1. Para cada câmera: envia a orientação do robô, `updateInputs`, `Logger.processInputs`.
2. **Filtros de qualidade** (rejeita com motivo registrado): desabilitada, desconectada, sem tags,
   poucas tags, tags longe demais, robô girando > 720°/s (MegaTag 2), pose fora do campo, pose NaN.
3. **Confiança:** `xy = base × distância² ÷ nºTags × stdDevScale`; θ muito alto (ignora rotação da
   visão).
4. **Prioridade** (`Mode.PRIORITY`, padrão): câmeras são agrupadas pelo número de prioridade
   (0 = principal). **Só o grupo de menor número que ainda "tem visão" envia dados**; os demais
   ficam em espera. "Perder a visão" só vale depois de `lossTimeout` (0,25 s) sem tags válidas, para
   evitar alternância. Mesma prioridade = contribuem juntas. `Mode.FUSE_ALL` ignora prioridade.
5. Devolve a lista de `VisionMeasurement` aceitas e mantém `CameraStatus` por câmera (`getStatuses`,
   `hasAnyVision`, `getActiveCameraNames`, `setCameraEnabled`).

### 4.4 Conexão com o resto
```
Limelight (NetworkTables) → LimelightCamera → VisionManager → VisionSubsystem
        → RobotState.addVisionMeasurement(pose, timestamp, stdDevs)
```
`VisionSubsystem` (em `subsystems/vision`) cria `LimelightCamera` (ou `SimulatedVisionCamera` em
simulação) via `addLimelight(config)`, e lê heading/velocidade angular do `RobotState` para o
MegaTag 2. Adicionar uma câmera = **uma linha** no `RobotContainer`.

---

## 5. `lib/navigation` — ir até uma coordenada desviando de obstáculos

### 5.1 Peças
| Classe | Função |
|---|---|
| `RectangleObstacle` | Retângulo alinhado aos eixos (2 cantos ou centro+dimensões); `inflated(m)`, `containsStrict`, `corners`. |
| `LiangBarsky` | Recorte segmento×retângulo. `intersectsInterior` ignora toque na borda/canto (essencial: os pontos de desvio ficam exatamente nos cantos). |
| `ObstacleField` | Guarda obstáculos e **planeja rotas** (abaixo). |
| `GoToConfig` | Limites, ganhos, tolerâncias, `flipForAlliance`. |
| `GoToPoseCommand` | O comando em si. |
| `AllianceUtil` | `isRed()`, `flipIfRed(Translation2d/Rotation2d/Pose2d)` via `FlippingUtil` do PathPlanner. |

### 5.2 Planejamento (`ObstacleField.planPath`)
1. Cada obstáculo é **inflado pelo raio do robô** → o robô vira um ponto.
2. Se `start→goal` está livre (Liang-Barsky) → rota = `[goal]`.
3. Senão: **grafo de visibilidade** — nós = start, goal e cantos dos obstáculos inflados (descarta
   cantos dentro de outro obstáculo ou fora do campo); arestas = segmentos sem colisão; **Dijkstra**
   devolve a rota mais curta. Arestas canto↔canto são cacheadas até os obstáculos mudarem.
4. Se o start/goal estiver dentro da folga de um obstáculo, aquela zona é testada **sem a folga**
   (o robô consegue sair sem atravessar o obstáculo real).
5. Sem rota (ex.: destino dentro do obstáculo) → `Optional.empty()`; o comando avisa no Driver
   Station e vai em linha reta.
6. `forCurrentAlliance()` devolve o campo espelhado quando estamos no vermelho.

### 5.3 `GoToPoseCommand` (ciclo)
- `initialize`: espelha destino/heading/obstáculos se `flipForAlliance`; planeja; reseta os
  controladores com a velocidade atual (sem tranco).
- `execute` (a cada 20 ms): **replaneja a partir da pose atual** (robusto a empurrões), descarta
  pontos já atingidos (< 0,15 m), calcula a velocidade com um **`ProfiledPIDController` sobre a
  distância restante da rota inteira** (parada suave no destino), aponta para o próximo ponto e
  comanda `swerve.runFieldVelocity(...)`.
- **Heading:** com `Translation2d` mantém o heading do início; com `Pose2d` gira suavemente (perfil
  trapezoidal) até o heading do destino.
- `isFinished`: posição (3 cm), heading (2°) e velocidade (5 cm/s) dentro da tolerância.
- `end`: `swerve.stop()`.

### 5.4 Conexão com o resto
`GoToPoseCommand` depende **só** de `SwerveSubsystem` (para `getPose`, `getFieldVelocity`,
`getChassisSpeeds`, `runFieldVelocity`, `stop`) e do `ObstacleField`. Os obstáculos são cadastrados
no `RobotContainer`. `Autos.goTo(...)` e os bindings criam o comando.

---

## 6. `RobotState` — a pose do robô (`frc.robot.RobotState`)

- Envolve um `SwerveDrivePoseEstimator` (rodas + gyro + visão) e uma **odometria pura** em paralelo
  (`getOdometryOnlyPose`) para você ver no log quanto a visão corrige
  (`getVisionCorrectionMeters`).
- **Entradas:** `addOdometryObservation(ts, gyroYaw, positions)` e `setRobotRelativeVelocity`
  (odometria das rodas) e `addVisionMeasurement(pose, ts, stdDevs)` ← `VisionSubsystem`.
  > ⚠️ **Hoje só a visão alimenta o `RobotState`:** o `SwerveSubsystem` atual não chama
  > `addOdometryObservation`/`setRobotRelativeVelocity`. Como o PathPlanner usa
  > `robotState::getPose` e `robotState::resetPose`, ele não recebe odometria das rodas até alguém
  > ligar isso (por exemplo, em `RobotContainer.logPeriodic()` com `m_swerve.getHeading()`,
  > `getModulePositions()` e `getChassisSpeeds()`). O `GoToPoseCommand` não é afetado: ele usa a
  > pose do próprio YAGSL (`swerve.getPose()`).
- **Saídas:** `getPose`, `getPoseAt(timestamp)`, `getHeading`, `getRobotRelativeVelocity`,
  `getFieldRelativeVelocity`, `hasRecentVisionFix`, `resetPose(...)`, `log()`.
- **Quem lê:** PathPlanner, `VisionSubsystem`
  (heading para MegaTag 2).
- A pose **nunca é espelhada** pela aliança.

---

## 7. AdvantageKit (logging e replay)

- `Robot` estende **`LoggedRobot`**. Modos em `Constants.kCurrentMode`: `REAL` (grava `.wpilog` no
  pendrive + NT4), `SIM` (NT4), `REPLAY` (reprocessa um log; troque `kSimMode`).
  `Logger.start()` é chamado **antes** de criar o `RobotContainer`.
- Os `*Inputs` (`MotorInputs`, `GyroInputs`, `AbsoluteEncoderInputs`, `DigitalSensorInputs`,
  `DistanceSensorInputs`, `VisionInputs`) implementam **`LoggableInputs` à mão** (`toLog`/`fromLog`),
  então o replay funciona **sem o plugin Gradle** e sem `@AutoLog`.
- **Regra do padrão:** `hardware.updateInputs(inputs)` → `Logger.processInputs("Chave", inputs)` →
  usar só `inputs` na lógica. No replay, `processInputs` sobrescreve `inputs` com os valores
  gravados, e a lógica reproduz o mesmo comportamento.
- Saídas com `Logger.recordOutput(...)`: setpoints/estados do swerve, poses aceitas pela visão,
  rota do GoTo, trajetória do PathPlanner, estado do `RobotState`, etc.
- Consequência: `lib.motors`, `lib.sensors` e `lib.vision` agora importam `org.littletonrobotics`.

---

## 8. Como tudo se conecta (fluxo por ciclo de 20 ms)

```
Robot.robotPeriodic()
 ├─ CommandScheduler.run()
 │   ├─ periodic() de cada subsistema (na ordem de criação no RobotContainer)
 │   │   ├─ SwerveSubsystem   : YAGSL updateOdometry() + log (pose, heading, estados dos módulos)
 │   │   ├─ VisionSubsystem   : VisionManager.update(heading, yawRate)
 │   │   │                       → RobotState.addVisionMeasurement(...) para cada medição aceita
 │   │   └─ Mecanismos        : intake / shooter / elevator / arm (updateInputs → log → lógica)
 │   └─ execute() dos comandos
 │        ├─ FieldDriveCommand / FreedomDriveCommand (piloto) → swerve.runVelocity
 │        ├─ GoToPoseCommand              → lê swerve.getPose (YAGSL) → swerve.runFieldVelocity
 │        └─ PathPlanner (AutoBuilder)    → lê RobotState.getPose → swerve.runVelocity
 └─ RobotContainer.logPeriodic() → RobotState.log()
```

### 8.1 Mapa de dependências (quem usa quem)
| Quem | Usa |
|---|---|
| `SwerveSubsystem` | **YAGSL** (`SwerveDrive`, lê `deploy/swerve/*.json`), `Constants.DriveConstants`, `AllianceUtil`, `Logger` |
| `VisionSubsystem` | `VisionManager`, `LimelightCamera`/`SimulatedVisionCamera`, `RobotState` |
| `RobotState` | WPILib (`SwerveDrivePoseEstimator`, `SwerveDriveOdometry`), `Logger` |
| `GoToPoseCommand` | `SwerveSubsystem`, `ObstacleField`, `AllianceUtil`, `GoToConfig` |
| `PathPlannerSetup` | `RobotState` (pose/reset), `SwerveSubsystem` (`getChassisSpeeds`, `runVelocity`, posição dos módulos), `AllianceUtil`, `Constants.DriveConstants` |
| `Autos` | PathPlanner (`AutoBuilder`, `NamedCommands`), `GoToPoseCommand` |
| `RobotContainer` | monta tudo: `RobotState`, `SwerveSubsystem`, `VisionSubsystem`, `ObstacleField`, `PathPlannerSetup`, `Autos`, bindings, seletor de autônomo |
| `Robot` | AdvantageKit (`Logger`, receivers), `RobotContainer` |
| `FieldDriveCommand`, `FreedomDriveCommand` | `SwerveSubsystem` (`runVelocity`, `getHeading`), `TeleopDriveInputs` |
| `TeleopDriveInputs` | `Constants.DriveConstants` (velocidades, zonas mortas, fração do RT) |
| `DriveModeSelector` | os dois comandos de direção; guarda o modo (FIELD/FREEDOM) e monta o comando padrão do swerve |
| `CollectorSubsystem`, `IntakePivotSubsystem`, `ShooterSubsystem` | `MotorFactory.createOrSim`, `SafeVelocityMotor`/`SafePositionMotor`, `Logger` (veja `subsystems/README.md`) |

### 8.2 Swerve (YAGSL + JSON em `deploy/swerve`)
O drivetrain **não usa** `lib.motors`/`lib.sensors`: o **YAGSL** controla os motores, encoders e o
gyro a partir da configuração em `src/main/deploy/swerve/` (IMU Pigeon 2 id 13; 4 módulos com
Kraken X60 na tração, NEO/SparkMax no esterço e CANcoder; offsets, relações, diâmetro da roda,
massa, limites de corrente e PID). **Para mudar o robô, edite os JSON**, não o código.

- `Constants.DriveConstants` guarda o que o JSON não traz: velocidade máxima (`kMaxSpeedMps`, também
  passada ao YAGSL), velocidade angular máxima e os parâmetros da teleoperação do piloto
  (`kStickDeadband`, `kBaseSpeedFraction`, `kRotationSpeedFraction`, `kBoostDeadband`).
- `SwerveSubsystem` é um invólucro fino do YAGSL: cria o `SwerveDrive` (`SwerveParser`) a partir do
  JSON, chama `updateOdometry()` a cada ciclo e registra pose, heading, velocidades e estados dos
  módulos no AdvantageKit. Expõe `runVelocity` (referencial do robô), `runFieldVelocity` (campo),
  `stop`, `lockWheels` (`lockPose()`), `zeroHeading()` (= `zeroGyro()`), `getPose()`,
  `getHeading()` (yaw do gyro), `getChassisSpeeds()`, `getFieldVelocity()`, `getKinematics()` e
  `getCircumradiusMeters()` (raio usado pelo `ObstacleField`).
- A direção pelo joystick fica nos comandos `FieldDriveCommand` (frente fixa = a de quando o robô
  foi ligado ou zerado) e `FreedomDriveCommand` (sem frente fixa: relativo ao robô), em
  `commands/swerve`. O modo FIELD usa o **yaw do gyro** (`getHeading()`), logo **não depende da
  aliança**; o START chama `zeroHeading()` e redefine a frente para onde o robô aponta.
- `driveCommand` (a direção antiga, em modo campo com inversão pela aliança) continua no
  `SwerveSubsystem`, mas o `RobotContainer` não o usa mais: o comando padrão agora é o dos dois modos.
- **YAGSL 2026.1.13** + `org.dyn4j:dyn4j` (no `build.gradle`, usado pela simulação do YAGSL).
- `controllerproperties.json` (PID de heading do YAGSL) só é usado se você usar os controles de
  heading do próprio YAGSL; hoje não é usado.

### 8.3 PathPlanner
`PathPlannerSetup.configure(swerve, robotState)` chama `AutoBuilder.configure` com: pose
(`RobotState::getPose`), reset (`RobotState::resetPose`), velocidade do chassi, saída
(`swerve.runVelocity`), `PPHolonomicDriveController`, `RobotConfig` **montado em código**
(massa, MOI, atrito, motor, relação, limite de corrente, e a posição dos módulos lida do YAGSL),
e `AllianceUtil::isRed` para espelhar trajetórias. Também liga o log da
trajetória ativa e da pose-alvo. `Autos` oferece `followPath`, `pathfindTo`, `goTo`,
`registerNamedCommands` e `exampleAuto`.

### 8.4 Alianças (um só lugar de verdade)
`AllianceUtil` → `FlippingUtil` do PathPlanner. Usado em: destino/heading/obstáculos
(`GoToPoseCommand`) e trajetórias do PathPlanner. Cadastre **tudo sempre no referencial azul**. O
joystick do modo FIELD **não** usa a aliança: a frente é a de quando o robô foi ligado (ou zerado
com o START).

---

## 9. Como estender

| Quero… | Faça |
|---|---|
| Adicionar uma Limelight | `.addLimelight(new VisionCameraConfig("nome").withPriority(n))` no `RobotContainer` |
| Trocar Kraken por NEO | Mudar `MotorType` na constante do subsistema (`TALON_FX` → `SPARK_MAX`/`SPARK_FLEX`) |
| Trocar Pigeon por NavX no swerve | Editar `imu.type` em `deploy/swerve/swervedrive.json` (o YAGSL cuida do gyro do drivetrain) |
| Mudar IDs, offsets, relações, PID do swerve | Editar os JSON em `deploy/swerve/` (e `modules/`) |
| Novo sensor (ex.: cor) | Criar interface + `*Inputs` (`LoggableInputs`) + real + simulada, e um método na `SensorFactory` — copie o padrão de `DistanceSensor` |
| Novo mecanismo | Subsistema com `MotorFactory.createOrSim` + sensores via `SensorFactory`; no `periodic()`: `updateInputs` → `Logger.processInputs` → lógica (veja `IntakeSubsystem`, `ShooterSubsystem`, `ElevatorSubsystem`, `ArmSubsystem` nos projetos `robo_*`) |
| Mecanismo com RPM (shooter, rolos) | `SafeVelocityMotor` (veja `ShooterSubsystem`) |
| Mecanismo com posição (pivô, braço) | `SafePositionMotor` (veja `IntakePivotSubsystem`) |
| Marcar uma zona proibida | `m_obstacles.addObstacle(x1, y1, x2, y2)` (azul, metros) no `RobotContainer` |
| Ir para um ponto | `new GoToPoseCommand(swerve, obstacles, new Translation2d(x, y))` ou com `Pose2d` para girar |
| Usar no PathPlanner | `NamedCommands.registerCommand("Nome", comando)` |

---

## 10. Armadilhas conhecidas

1. `updateInputs` **uma vez por ciclo** (ver seção 2).
2. **Nunca reutilize a mesma instância de `Command` em duas composições** (o WPILib lança exceção);
   por isso `ElevatorSubsystem` usa métodos que criam comandos novos. Cuidado também com
   `NamedCommands.registerCommand`: a mesma instância é devolvida a cada uso.
3. Encoder absoluto de esterço/pivô precisa medir o eixo do mecanismo **1:1**.
4. Encoder de motor é **relativo**: elevadores precisam de homing (chave de fim de curso).
5. Em replay, o código **não pode** ler hardware fora de `updateInputs`/`processInputs`.
6. Pose e obstáculos no referencial **azul**; não espelhe manualmente.
7. **Unidades dos ganhos entre fabricantes:** `PIDConfig`/`MotorController.setVelocity/setPosition`
   usam o PID interno de cada controlador, e as unidades de P/I/D variam por fabricante e firmware
   (a documentação da REVLib 2026 não as deixa claras). Para ficar previsível e igual na simulação,
   os mecanismos novos usam `SafeVelocityMotor`/`SafePositionMotor` (controle em volts).
8. Controle em volts exige **encoder e motor no mesmo sentido**; o guarda de
   `SafePositionMotor` (`WRONG_DIRECTION`) existe para o caso de estarem trocados.

---

## 11. O que ainda falta / pontos de risco

- **Compilação:** verificada (ver o status no topo). `SparkMotor` usa APIs da REVLib marcadas como
  *deprecated/for removal* (`velocityFF`, `setReference`, `ResetMode`...): compilam hoje, mas podem
  sumir em versões futuras.
- **Configurar a Limelight:** hostname (igual ao nome no código) e posição da câmera no robô, na
  interface web de cada câmera.
- **`RobotState` sem odometria:** veja o aviso da seção 6 (o PathPlanner usa a pose do `RobotState`).
- **JSON do swerve:** os `pidfproperties.json` têm o mesmo valor (0,00206545) para tração (Kraken) e
  esterço (NEO); é típico de valor padrão e provavelmente precisa de ajuste, principalmente no
  esterço. Confira também as unidades do YAGSL (polegadas, libras, graus).
- **`Constants.DriveConstants` e `PathPlannerSetup`:** `kMaxSpeedMps` e os valores do `RobotConfig`
  (massa 62 kg, MOI 5, roda de 2", relação 6,75...) são estimativas fixas no código; confira com o
  robô e com `deploy/swerve/modules/physicalproperties.json`.
- **YAGSL:** o projeto usa a **2026.1.13**. As versões a partir da 2026.8.25 (inclusive a
  2026.10.03) mudaram de arquitetura: viraram uma camada fina sobre o **YAMS** e **não têm mais**
  `swervelib.SwerveDrive`, então o `SwerveSubsystem` (API clássica: `new SwerveParser(File)
  .createSwerveDrive(...)`) não compila com elas. Não atualize o YAGSL pelo gerenciador de
  vendordeps sem migrar o `SwerveSubsystem` para a API nova (que exige o vendordep do YAMS).
- O drivetrain não usa `lib.motors`/`lib.sensors` (só os mecanismos usam). `Pigeon2Gyro`,
  `NavXGyro` e `SimulatedGyro` continuam na lib para outros usos.
- `Constants.IntakeConstants` (rascunho antigo, CAN 23) não é usado: o coletor novo usa
  `subsystems/collector/CollectorConstants` (mesmo ID 23) e o pivô, `subsystems/intake/IntakeConstants`.
- **Obstáculos reais** ainda não cadastrados.
- **Pathfinding logável:** copiar `LocalADStarAK.java` do template do AdvantageKit se quiser o
  `pathfindTo` no log (o `GoTo` não depende disso).
