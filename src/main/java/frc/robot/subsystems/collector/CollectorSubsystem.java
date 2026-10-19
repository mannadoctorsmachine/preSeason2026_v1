package frc.robot.subsystems.collector;

import static frc.robot.subsystems.collector.CollectorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.motors.MotorConfig;
import frc.robot.lib.motors.MotorController;
import frc.robot.lib.motors.MotorFactory;
import frc.robot.lib.motors.MotorInputs;
import frc.robot.lib.motors.NeutralMode;
import org.littletonrobotics.junction.Logger;

/**
 * Sistema de coleta: um rolo com 1 motor NEO Vortex. Este subsistema so cuida do HARDWARE (motor,
 * leituras e log). Quem decide quando girar e o comando
 * {@link frc.robot.commands.collector.CollectCommand}.
 */
public class CollectorSubsystem extends SubsystemBase {
  private final MotorController roller;
  private final MotorInputs inputs = new MotorInputs();
  private boolean running = false;

  public CollectorSubsystem() {
    roller =
        MotorFactory.createOrSim(
            kRollerCanId,
            kCanBus,
            kMotorType,
            new MotorConfig()
                .withInverted(kRollerInverted)
                .withNeutralMode(NeutralMode.COAST)
                .withGearRatio(kRollerGearRatio)
                .withStatorCurrentLimit(kRollerCurrentLimitAmps)
                .withSupplyCurrentLimit(kRollerCurrentLimitAmps));
  }

  @Override
  public void periodic() {
    roller.updateInputs(inputs);
    Logger.processInputs("Collector/Roller", inputs);
    Logger.recordOutput("Collector/Running", running);
  }

  /** Aplica uma saida percentual (-1..1) ao rolo. */
  public void setOutput(double percent) {
    running = percent != 0.0;
    roller.setPercentOutput(percent);
  }

  public void stop() {
    running = false;
    roller.stop();
  }

  public boolean isRunning() {
    return running;
  }
}
