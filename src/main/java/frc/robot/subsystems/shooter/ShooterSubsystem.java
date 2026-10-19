package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.motors.MotorConfig;
import frc.robot.lib.motors.MotorController;
import frc.robot.lib.motors.MotorFactory;
import frc.robot.lib.motors.NeutralMode;
import frc.robot.lib.motors.SafeVelocityMotor;
import org.littletonrobotics.junction.Logger;

/**
 * Conjunto Index -> Intermediario -> Shooter, acionado todo junto pelo botao RT do operador.
 *
 * <h2>Como funciona (cascata)</h2>
 * <ol>
 *   <li>RT apertado: o INDEX comeca a girar e sobe (com rampa) ate 5000 RPM, sem passar disso.
 *   <li>Quando o Index chega ao "RPM x" ({@link ShooterConstants#kReachedFraction} do alvo), o
 *       INTERMEDIARIO comeca a girar ate 5000 RPM.
 *   <li>Quando o Intermediario chega ao "RPM x", o SHOOTER comeca a girar ate 6000 RPM.
 *   <li>Soltar o RT para tudo. Os estagios, uma vez ligados, nao desligam sozinhos se o RPM cair.
 * </ol>
 *
 * <h2>Seguranca (veja {@link SafeVelocityMotor})</h2>
 * Cada estagio compara o RPM em MODULO (funciona com o encoder lendo positivo ou negativo), tem teto
 * de RPM, rampa, deteccao de overspeed, de travamento/encoder morto e de leitura invalida. Se
 * qualquer estagio falhar, TODOS param e a sequencia so recomeca em um novo aperto do RT (que
 * limpa as falhas). Alem disso, se o Index ou o Intermediario nao atingirem o RPM em
 * {@link ShooterConstants#kStageTimeoutSeconds}, a sequencia aborta.
 */
public class ShooterSubsystem extends SubsystemBase {
  /** Fase atual da sequencia (aparece no log e no dashboard). */
  public enum Phase {
    IDLE,
    SPINNING_INDEX,
    SPINNING_INTERMEDIATE,
    SPINNING_SHOOTER,
    READY,
    FAULT
  }

  private static final double kDtSeconds = 0.02;

  private final SafeVelocityMotor index;
  private final SafeVelocityMotor intermediate;
  private final SafeVelocityMotor shooter;

  private Phase phase = Phase.IDLE;
  private boolean intermediateStarted = false;
  private boolean shooterStarted = false;
  private double phaseSeconds = 0.0;

  public ShooterSubsystem() {
    index =
        createStage(
            "Shooter/Index",
            kIndexCanId,
            kIndexDirection,
            kIndexGearRatio,
            kIndexTargetRpm,
            kIndexCurrentLimitAmps,
            NeutralMode.BRAKE);
    intermediate =
        createStage(
            "Shooter/Intermediate",
            kIntermediateCanId,
            kIntermediateDirection,
            kIntermediateGearRatio,
            kIntermediateTargetRpm,
            kIntermediateCurrentLimitAmps,
            NeutralMode.BRAKE);
    shooter =
        createStage(
            "Shooter/Shooter",
            kShooterCanId,
            kShooterDirection,
            kShooterGearRatio,
            kShooterTargetRpm,
            kShooterCurrentLimitAmps,
            NeutralMode.COAST);
  }

  private static SafeVelocityMotor createStage(
      String name,
      int canId,
      int direction,
      double gearRatio,
      double maxRpm,
      double currentLimitAmps,
      NeutralMode neutralMode) {
    MotorController motor =
        MotorFactory.createOrSim(
            canId,
            kCanBus,
            kMotorType,
            new MotorConfig()
                .withNeutralMode(neutralMode)
                .withGearRatio(gearRatio)
                .withStatorCurrentLimit(currentLimitAmps)
                .withSupplyCurrentLimit(currentLimitAmps));

    return new SafeVelocityMotor(
        name,
        motor,
        new SafeVelocityMotor.Config(maxRpm)
            .withDirection(direction)
            .withFeedforward(kS, kvForGearRatio(gearRatio))
            .withGains(kPVoltsPerRpm, kIVoltsPerRpmSecond)
            .withMaxVolts(kMaxVolts)
            .withRamp(kRampRpmPerSec)
            .withOverspeed(kSoftOverspeedFraction, kHardOverspeedFraction)
            .withStallDetection(kStallMinVolts, kStallMaxRpm, kStallSeconds));
  }

  @Override
  public void periodic() {
    index.periodic();
    intermediate.periodic();
    shooter.periodic();

    // Falha em qualquer estagio: para todos (inclusive os que estao bem)
    if (anyFault() && phase != Phase.FAULT) {
      abort("falha em " + faultedStageName());
    }

    Logger.recordOutput("Shooter/Phase", phase.name());
    Logger.recordOutput("Shooter/IntermediateStarted", intermediateStarted);
    Logger.recordOutput("Shooter/ShooterStarted", shooterStarted);
    SmartDashboard.putString("Shooter", phase.name());
    SmartDashboard.putBoolean("Shooter Pronto", isReady());
  }

  // ---------------------------------------------------------------------------------------------
  // Sequencia
  // ---------------------------------------------------------------------------------------------

  /** Inicio de um novo aperto do RT: limpa falhas antigas e recomeca do Index. */
  private void startSequence() {
    index.clearFault();
    intermediate.clearFault();
    shooter.clearFault();
    intermediateStarted = false;
    shooterStarted = false;
    phaseSeconds = 0.0;
    phase = Phase.SPINNING_INDEX;
  }

  /** Executado a cada ciclo enquanto o RT estiver apertado. */
  private void runSequence() {
    if (phase == Phase.FAULT) {
      return; // fica parado ate soltar o RT e apertar de novo
    }
    phaseSeconds += kDtSeconds;

    // Estagio 1: o Index sempre pede o alvo (o controle limita a 5000 RPM e sobe com rampa)
    index.setTargetRpm(kIndexTargetRpm);

    // Estagio 2: Intermediario liga quando o Index atinge o "RPM x"
    if (!intermediateStarted && index.hasReached(kReachedFraction)) {
      intermediateStarted = true;
      phase = Phase.SPINNING_INTERMEDIATE;
      phaseSeconds = 0.0;
    }
    if (intermediateStarted) {
      intermediate.setTargetRpm(kIntermediateTargetRpm);
    }

    // Estagio 3: Shooter liga quando o Intermediario atinge o "RPM x"
    if (intermediateStarted && !shooterStarted && intermediate.hasReached(kReachedFraction)) {
      shooterStarted = true;
      phase = Phase.SPINNING_SHOOTER;
      phaseSeconds = 0.0;
    }
    if (shooterStarted) {
      shooter.setTargetRpm(kShooterTargetRpm);
      if (shooter.hasReached(kReachedFraction)) {
        phase = Phase.READY;
      } else if (phase == Phase.READY) {
        phase = Phase.SPINNING_SHOOTER;
      }
    }

    // Tempo maximo para o Index e o Intermediario atingirem o RPM
    if ((phase == Phase.SPINNING_INDEX || phase == Phase.SPINNING_INTERMEDIATE)
        && phaseSeconds > kStageTimeoutSeconds) {
      abort(
          (phase == Phase.SPINNING_INDEX ? "Index" : "Intermediario")
              + " nao atingiu o RPM em "
              + kStageTimeoutSeconds
              + " s");
    }
  }

  private void abort(String reason) {
    DriverStation.reportError("[Shooter] sequencia abortada: " + reason, false);
    index.stop();
    intermediate.stop();
    shooter.stop();
    intermediateStarted = false;
    shooterStarted = false;
    phase = Phase.FAULT;
  }

  /** Para tudo imediatamente (soltar o RT, desabilitar). */
  public void stopAll() {
    index.stop();
    intermediate.stop();
    shooter.stop();
    intermediateStarted = false;
    shooterStarted = false;
    phaseSeconds = 0.0;
    // Com falha latched o dashboard continua mostrando FAULT ate o proximo aperto do RT
    phase = anyFault() ? Phase.FAULT : Phase.IDLE;
  }

  // ---------------------------------------------------------------------------------------------
  // Leituras
  // ---------------------------------------------------------------------------------------------

  private boolean anyFault() {
    return index.isFaulted() || intermediate.isFaulted() || shooter.isFaulted();
  }

  private String faultedStageName() {
    if (index.isFaulted()) {
      return index.getName() + " (" + index.getFault().description + ")";
    }
    if (intermediate.isFaulted()) {
      return intermediate.getName() + " (" + intermediate.getFault().description + ")";
    }
    return shooter.getName() + " (" + shooter.getFault().description + ")";
  }

  public Phase getPhase() {
    return phase;
  }

  /** @return true se o Shooter ja esta no RPM (e o resto da cascata girando). */
  public boolean isReady() {
    return phase == Phase.READY;
  }

  public double getIndexRpm() {
    return index.getSpeedRpm();
  }

  public double getIntermediateRpm() {
    return intermediate.getSpeedRpm();
  }

  public double getShooterRpm() {
    return shooter.getSpeedRpm();
  }

  // ---------------------------------------------------------------------------------------------
  // Comandos
  // ---------------------------------------------------------------------------------------------

  /** Roda a cascata Index -> Intermediario -> Shooter enquanto o comando estiver ativo (RT). */
  public Command shootCommand() {
    return runEnd(this::runSequence, this::stopAll)
        .beforeStarting(this::startSequence)
        .withName("ShootSequence");
  }
}
