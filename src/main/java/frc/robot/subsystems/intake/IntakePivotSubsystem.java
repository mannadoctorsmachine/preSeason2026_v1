package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.motors.MotorConfig;
import frc.robot.lib.motors.MotorController;
import frc.robot.lib.motors.MotorFactory;
import frc.robot.lib.motors.NeutralMode;
import frc.robot.lib.motors.SafePositionMotor;
import org.littletonrobotics.junction.Logger;

/**
 * Pivo do intake: 1 motor NEO Vortex que so desce (DOWN) e sobe (UP), com controle PID + perfil de
 * movimento e protecoes (veja {@link SafePositionMotor}).
 *
 * <p>Protecao contra clique duplo: o subsistema guarda para onde o intake esta indo e IGNORA
 * pedidos repetidos (LB duas vezes, ou LB com o intake ja abaixado) e pedidos que chegam logo depois
 * de um pedido aceito ({@link IntakeConstants#kRequestCooldownSeconds}). Nesses casos nada acontece.
 */
public class IntakePivotSubsystem extends SubsystemBase {
  /** Posicoes possiveis do intake. */
  public enum Position {
    UP,
    DOWN
  }

  private final SafePositionMotor pivot;

  private Position target = Position.UP;
  private double lastAcceptedRequestSeconds = Double.NEGATIVE_INFINITY;

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
  // Pedidos (com protecao contra clique duplo)
  // ---------------------------------------------------------------------------------------------

  /**
   * Pede uma posicao. Retorna false (e nao faz nada) se o pedido for repetido ou chegar durante o
   * tempo de espera; um pedido da mesma posicao so e aceito de novo se o pivo estiver em falha (para
   * permitir tentar outra vez).
   *
   * @return true se o pedido foi aceito.
   */
  public boolean request(Position requested) {
    double now = Timer.getFPGATimestamp();

    if (requested == target && !pivot.isFaulted()) {
      Logger.recordOutput("Intake/Pivot/LastRequest", requested.name() + " ignorado: ja esta indo/em " + requested.name());
      return false;
    }
    if (now - lastAcceptedRequestSeconds < kRequestCooldownSeconds) {
      Logger.recordOutput("Intake/Pivot/LastRequest", requested.name() + " ignorado: clique duplo (espera)");
      return false;
    }

    target = requested;
    lastAcceptedRequestSeconds = now;
    pivot.clearFault();
    pivot.setGoalRotations(toRotations(requested));
    Logger.recordOutput("Intake/Pivot/LastRequest", requested.name() + " aceito");
    return true;
  }

  private static double toRotations(Position position) {
    return ((position == Position.DOWN) ? kDownDegrees : kUpDegrees) / 360.0;
  }

  // ---------------------------------------------------------------------------------------------
  // Comandos
  // ---------------------------------------------------------------------------------------------

  /** Desce o intake (LB). Instantaneo: o subsistema continua controlando a posicao sozinho. */
  public Command lowerCommand() {
    return runOnce(() -> request(Position.DOWN)).withName("IntakeLower");
  }

  /** Sobe o intake (RB). Instantaneo: o subsistema continua controlando a posicao sozinho. */
  public Command raiseCommand() {
    return runOnce(() -> request(Position.UP)).withName("IntakeRaise");
  }

  // ---------------------------------------------------------------------------------------------
  // Leituras
  // ---------------------------------------------------------------------------------------------

  public double getAngleDegrees() {
    return pivot.getPositionRotations() * 360.0;
  }

  public Position getTarget() {
    return target;
  }

  /** @return true se o intake chegou na posicao pedida. */
  public boolean atTarget() {
    return pivot.atGoal();
  }

  public boolean isFaulted() {
    return pivot.isFaulted();
  }

  private String describe() {
    if (pivot.isFaulted()) {
      return "FALHA: " + pivot.getFault().description;
    }
    String state = pivot.atGoal() ? "" : (target == Position.DOWN ? "descendo" : "subindo");
    return target.name() + (state.isEmpty() ? "" : " (" + state + ")");
  }
}
