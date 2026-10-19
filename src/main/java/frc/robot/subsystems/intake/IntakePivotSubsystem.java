package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.motors.MotorConfig;
import frc.robot.lib.motors.MotorController;
import frc.robot.lib.motors.MotorFactory;
import frc.robot.lib.motors.MotorFault;
import frc.robot.lib.motors.NeutralMode;
import frc.robot.lib.motors.SafePositionMotor;
import org.littletonrobotics.junction.Logger;

/**
 * Pivo do intake: 1 motor NEO Vortex que so desce (DOWN) e sobe (UP), com controle PID + perfil de
 * movimento e protecoes (veja {@link SafePositionMotor}). Este subsistema so cuida do HARDWARE e
 * do estado; a decisao de aceitar ou ignorar um pedido (protecao contra clique duplo) e do comando
 * {@link frc.robot.commands.intake.MoveIntakeCommand}.
 */
public class IntakePivotSubsystem extends SubsystemBase {
  /** Posicoes possiveis do intake. */
  public enum Position {
    UP,
    DOWN
  }

  private final SafePositionMotor pivot;

  private Position target = Position.UP;
  private double lastTargetChangeSeconds = Double.NEGATIVE_INFINITY;

  public IntakePivotSubsystem() {
    MotorController motor =
        MotorFactory.createOrSim(
            kPivotCanId,
            kCanBus,
            kMotorType,
            new MotorConfig()
                .withInverted(kPivotInverted)
                .withNeutralMode(NeutralMode.BRAKE)
                .withGearRatio(kPivotGearRatio)
                .withStatorCurrentLimit(kPivotCurrentLimitAmps)
                .withSupplyCurrentLimit(kPivotCurrentLimitAmps));

    pivot =
        new SafePositionMotor(
            "Intake/Pivot",
            motor,
            new SafePositionMotor.Config(kUpDegrees / 360.0, kDownDegrees / 360.0)
                .withGains(kPivotKp, kPivotKd)
                .withFeedforward(kPivotKs, kPivotKv)
                .withGravity(kPivotKg, 0.0)
                .withMaxVolts(kPivotMaxVolts)
                .withProfile(kPivotMaxVelocityRotPerSec, kPivotMaxAccelerationRotPerSecSq)
                .withTolerance(kPivotToleranceDegrees / 360.0));

    // O robo liga com o intake recolhido: o encoder relativo assume a posicao UP
    pivot.resetPosition(kUpDegrees / 360.0);
  }

  @Override
  public void periodic() {
    pivot.periodic();

    Logger.recordOutput("Intake/Pivot/Target", target.name());
    Logger.recordOutput("Intake/Pivot/AngleDegrees", getAngleDegrees());
    SmartDashboard.putString("Intake", describe());
  }

  // ---------------------------------------------------------------------------------------------
  // Acoes
  // ---------------------------------------------------------------------------------------------

  /**
   * Define a nova posicao do intake (limpa uma falha anterior e inicia o movimento). Nao faz
   * nenhuma verificacao de clique duplo: isso e do comando.
   */
  public void setTarget(Position newTarget) {
    target = newTarget;
    lastTargetChangeSeconds = Timer.getFPGATimestamp();
    pivot.clearFault();
    pivot.setGoalRotations(toRotations(newTarget));
  }

  private static double toRotations(Position position) {
    return ((position == Position.DOWN) ? kDownDegrees : kUpDegrees) / 360.0;
  }

  // ---------------------------------------------------------------------------------------------
  // Leituras
  // ---------------------------------------------------------------------------------------------

  public Position getTarget() {
    return target;
  }

  /** @return Segundos desde a ultima vez que o alvo foi definido. */
  public double getSecondsSinceTargetChange() {
    return Timer.getFPGATimestamp() - lastTargetChangeSeconds;
  }

  public double getAngleDegrees() {
    return pivot.getPositionRotations() * 360.0;
  }

  /** @return true se o intake chegou na posicao pedida. */
  public boolean atTarget() {
    return pivot.atGoal();
  }

  public boolean isFaulted() {
    return pivot.isFaulted();
  }

  public MotorFault getFault() {
    return pivot.getFault();
  }

  private String describe() {
    if (pivot.isFaulted()) {
      return "FALHA: " + pivot.getFault().description;
    }
    String state = pivot.atGoal() ? "" : (target == Position.DOWN ? "descendo" : "subindo");
    return target.name() + (state.isEmpty() ? "" : " (" + state + ")");
  }
}
