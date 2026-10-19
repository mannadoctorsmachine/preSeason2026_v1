package frc.robot.commands.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Cascata Index -> Intermediario -> Shooter, enquanto o comando estiver ativo (RT do operador).
 *
 * <ol>
 *   <li>O INDEX sobe (com rampa) ate 5000 RPM, sem passar disso.
 *   <li>Quando o Index chega ao "RPM x" ({@code kReachedFraction} do alvo), o INTERMEDIARIO liga e
 *       sobe ate 5000 RPM.
 *   <li>Quando o Intermediario chega ao "RPM x", o SHOOTER liga e sobe ate 6000 RPM.
 *   <li>Soltar o RT para tudo. Um estagio, depois de ligado, nao desliga se o RPM cair.
 * </ol>
 *
 * Seguranca: se qualquer estagio falhar (o subsistema ja para todos), a sequencia fica em FAULT ate
 * soltar o RT e apertar de novo (isso limpa as falhas). Se o Index ou o Intermediario nao atingirem
 * o RPM em {@code kStageTimeoutSeconds}, a sequencia aborta.
 */
public class ShootSequenceCommand extends Command {
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

  private final ShooterSubsystem shooter;

  private Phase phase = Phase.IDLE;
  private boolean intermediateStarted = false;
  private boolean shooterStarted = false;
  private double phaseSeconds = 0.0;

  public ShootSequenceCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    setName("ShootSequence");
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Novo aperto do RT: limpa falhas antigas e recomeca do Index
    shooter.clearFaults();
    intermediateStarted = false;
    shooterStarted = false;
    phaseSeconds = 0.0;
    setPhase(Phase.SPINNING_INDEX);
  }

  @Override
  public void execute() {
    if (shooter.hasFault()) {
      // O subsistema ja parou todos os estagios; fica parado ate soltar o RT
      setPhase(Phase.FAULT);
      return;
    }
    phaseSeconds += kDtSeconds;

    // Estagio 1: o Index sempre pede o alvo (o controle limita a 5000 RPM e sobe com rampa)
    shooter.setIndexRpm(kIndexTargetRpm);

    // Estagio 2: Intermediario liga quando o Index atinge o "RPM x"
    if (!intermediateStarted && shooter.indexReached(kReachedFraction)) {
      intermediateStarted = true;
      phaseSeconds = 0.0;
      setPhase(Phase.SPINNING_INTERMEDIATE);
    }
    if (intermediateStarted) {
      shooter.setIntermediateRpm(kIntermediateTargetRpm);
    }

    // Estagio 3: Shooter liga quando o Intermediario atinge o "RPM x"
    if (intermediateStarted && !shooterStarted && shooter.intermediateReached(kReachedFraction)) {
      shooterStarted = true;
      phaseSeconds = 0.0;
      setPhase(Phase.SPINNING_SHOOTER);
    }
    if (shooterStarted) {
      shooter.setShooterRpm(kShooterTargetRpm);
      if (shooter.shooterReached(kReachedFraction)) {
        setPhase(Phase.READY);
      } else if (phase == Phase.READY) {
        setPhase(Phase.SPINNING_SHOOTER);
      }
    }

    // Tempo maximo para o Index e o Intermediario atingirem o RPM
    if ((phase == Phase.SPINNING_INDEX || phase == Phase.SPINNING_INTERMEDIATE)
        && phaseSeconds > kStageTimeoutSeconds) {
      DriverStation.reportError(
          "[Shooter] sequencia abortada: "
              + (phase == Phase.SPINNING_INDEX ? "Index" : "Intermediario")
              + " nao atingiu o RPM em "
              + kStageTimeoutSeconds
              + " s",
          false);
      shooter.stopAll();
      intermediateStarted = false;
      shooterStarted = false;
      setPhase(Phase.FAULT);
    }
  }

  @Override
  public boolean isFinished() {
    return false; // roda enquanto o RT estiver apertado
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    intermediateStarted = false;
    shooterStarted = false;
    phaseSeconds = 0.0;
    // Com falha latched o dashboard continua mostrando FAULT ate o proximo aperto do RT
    setPhase(shooter.hasFault() ? Phase.FAULT : Phase.IDLE);
  }

  private void setPhase(Phase newPhase) {
    phase = newPhase;
    Logger.recordOutput("Shooter/Phase", phase.name());
    Logger.recordOutput("Shooter/IntermediateStarted", intermediateStarted);
    Logger.recordOutput("Shooter/ShooterStarted", shooterStarted);
    SmartDashboard.putString("Shooter", phase.name());
    SmartDashboard.putBoolean("Shooter Pronto", phase == Phase.READY);
  }

  /** @return Fase atual da sequencia. */
  public Phase getPhase() {
    return phase;
  }

  /** @return true se o Shooter ja esta no RPM (e o resto da cascata girando). */
  public boolean isReady() {
    return phase == Phase.READY;
  }
}
