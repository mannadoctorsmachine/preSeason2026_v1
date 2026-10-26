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
 * Pivo do intake: 1 motor NEO Vortex que so abre (OPEN) e fecha (CLOSED), com controle PID + perfil
 * de movimento e protecoes (veja {@link SafePositionMotor}). As posicoes sao o valor do encoder
 * primario (rotacoes do motor): fechado = {@code kClosedPosition}, aberto = {@code kOpenPosition}
 * (veja {@link IntakeConstants}). Este subsistema so cuida do HARDWARE e
 * do estado; a decisao de aceitar ou ignorar um pedido (protecao contra clique duplo) e do comando
 * {@link frc.robot.commands.intake.MoveIntakeCommand}.
 */
public class IntakePivotSubsystem extends SubsystemBase {
  /** Posicoes possiveis do intake. */
  public enum Position {
    CLOSED,
    OPEN
  }

  private final SafePositionMotor pivot;

  private Position target = Position.CLOSED;
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
                .withGearRatio(1.0) // posicao = encoder primario (rotacoes do motor)
                .withStatorCurrentLimit(kPivotCurrentLimitAmps)
                .withSupplyCurrentLimit(kPivotCurrentLimitAmps));

    SafePositionMotor.Config config =
        new SafePositionMotor.Config(kClosedPosition, kOpenPosition)
            .withGains(kPivotKp, kPivotKd)
            .withFeedforward(kPivotKs, kPivotKv)
            .withGravity(kPivotKg, 0.0)
            .withMaxVolts(kPivotMaxVolts)
            .withProfile(kPivotMaxVelocityRotPerSec, kPivotMaxAccelerationRotPerSecSq)
            .withTolerance(kPivotTolerance);
    // Limiares do guarda de seguranca em rotacoes do motor (o curso total e pequeno)
    config.wrongDirectionRotations = 0.05;
    config.stallProgressRotations = 0.02;
    config.graceVelocityRotPerSec = 1.0;

    pivot = new SafePositionMotor("Intake/Pivot", motor, config);

    // O robo liga com o intake FECHADO: o encoder relativo assume o valor de fechado
    pivot.resetPosition(kClosedPosition);
  }

  @Override
  public void periodic() {
    pivot.periodic();

    Logger.recordOutput("Intake/Pivot/Target", target.name());
    Logger.recordOutput("Intake/Pivot/Position", getPosition());
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
    return (position == Position.OPEN) ? kOpenPosition : kClosedPosition;
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

  /** @return Posicao do encoder primario (rotacoes do motor), o mesmo valor do REV Client. */
  public double getPosition() {
    return pivot.getPositionRotations();
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
    String state = pivot.atGoal() ? "" : (target == Position.OPEN ? "abrindo" : "fechando");
    return target.name() + (state.isEmpty() ? "" : " (" + state + ")");
  }
}
