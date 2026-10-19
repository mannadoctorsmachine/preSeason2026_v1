package frc.robot.subsystems.collector;

import static frc.robot.subsystems.collector.CollectorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.motors.MotorConfig;
import frc.robot.lib.motors.MotorController;
import frc.robot.lib.motors.MotorFactory;
import frc.robot.lib.motors.MotorInputs;
import frc.robot.lib.motors.NeutralMode;
import org.littletonrobotics.junction.Logger;

/**
 * Sistema de coleta: um rolo com 1 motor NEO Vortex. Enquanto o comando estiver ativo (botao LT do
 * operador segurado) o motor gira no sentido anti-horario; ao soltar, para.
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

  /** Gira o rolo (anti-horario) enquanto o comando estiver ativo; para ao terminar/cancelar. */
  public Command collectCommand() {
    return runEnd(
            () -> {
              running = true;
              roller.setPercentOutput(kCollectOutput);
            },
            this::stop)
        .withName("Collect");
  }

  public void stop() {
    running = false;
    roller.stop();
  }

  public boolean isRunning() {
    return running;
  }
}
