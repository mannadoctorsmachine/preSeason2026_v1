package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.motors.MotorConfig;
import frc.robot.lib.motors.MotorController;
import frc.robot.lib.motors.MotorFactory;
import frc.robot.lib.motors.NeutralMode;
import frc.robot.lib.motors.SafeVelocityMotor;
import org.littletonrobotics.junction.Logger;

/**
 * Conjunto Index, Intermediario e Shooter (3 motores). Este subsistema so cuida do HARDWARE: cada
 * estagio e um {@link SafeVelocityMotor} (RPM em modulo, teto, rampa e protecoes) e o subsistema
 * para TODOS os estagios se qualquer um falhar.
 *
 * <p>A ORDEM em que os estagios ligam (Index -> Intermediario -> Shooter, cada um quando o anterior
 * atinge o "RPM x") e do comando {@link frc.robot.commands.shooter.ShootSequenceCommand}.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final SafeVelocityMotor index;
  private final SafeVelocityMotor intermediate;
  private final SafeVelocityMotor shooter;

  private boolean faultHandled = false;

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

    // Seguranca: falha em qualquer estagio para todos (inclusive os que estao bem)
    if (hasFault() && !faultHandled) {
      faultHandled = true;
      DriverStation.reportError("[Shooter] falha em " + getFaultDescription() + ": parando tudo", false);
      stopAll();
    }
    Logger.recordOutput("Shooter/HasFault", hasFault());
  }

  // ---------------------------------------------------------------------------------------------
  // Acoes
  // ---------------------------------------------------------------------------------------------

  /** Define o alvo do Index (RPM em modulo; o teto e do proprio estagio). */
  public void setIndexRpm(double rpm) {
    index.setTargetRpm(rpm);
  }

  public void setIntermediateRpm(double rpm) {
    intermediate.setTargetRpm(rpm);
  }

  public void setShooterRpm(double rpm) {
    shooter.setTargetRpm(rpm);
  }

  /** Para os tres estagios imediatamente. */
  public void stopAll() {
    index.stop();
    intermediate.stop();
    shooter.stop();
  }

  /** Libera as falhas travadas (chamado pelo comando quando o operador aperta de novo). */
  public void clearFaults() {
    index.clearFault();
    intermediate.clearFault();
    shooter.clearFault();
    faultHandled = false;
  }

  // ---------------------------------------------------------------------------------------------
  // Leituras
  // ---------------------------------------------------------------------------------------------

  /** @return true se o Index atingiu {@code fraction} (0..1) do alvo pedido (em modulo). */
  public boolean indexReached(double fraction) {
    return index.hasReached(fraction);
  }

  public boolean intermediateReached(double fraction) {
    return intermediate.hasReached(fraction);
  }

  public boolean shooterReached(double fraction) {
    return shooter.hasReached(fraction);
  }

  public boolean hasFault() {
    return index.isFaulted() || intermediate.isFaulted() || shooter.isFaulted();
  }

  public String getFaultDescription() {
    if (index.isFaulted()) {
      return index.getName() + " (" + index.getFault().description + ")";
    }
    if (intermediate.isFaulted()) {
      return intermediate.getName() + " (" + intermediate.getFault().description + ")";
    }
    if (shooter.isFaulted()) {
      return shooter.getName() + " (" + shooter.getFault().description + ")";
    }
    return "nenhuma";
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
}
