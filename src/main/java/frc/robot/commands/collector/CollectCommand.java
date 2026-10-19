package frc.robot.commands.collector;

import static frc.robot.subsystems.collector.CollectorConstants.kCollectOutput;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;

/**
 * Coleta: enquanto o comando estiver ativo (LT do operador segurado) o rolo gira no sentido
 * anti-horario; ao soltar (ou cancelar) o rolo para.
 */
public class CollectCommand extends Command {
  private final CollectorSubsystem collector;

  public CollectCommand(CollectorSubsystem collector) {
    this.collector = collector;
    setName("Collect");
    addRequirements(collector);
  }

  @Override
  public void execute() {
    // Reaplicado a cada ciclo
    collector.setOutput(kCollectOutput);
  }

  @Override
  public void end(boolean interrupted) {
    collector.stop();
  }
}
