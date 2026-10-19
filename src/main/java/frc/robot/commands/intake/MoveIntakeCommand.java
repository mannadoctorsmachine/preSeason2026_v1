package frc.robot.commands.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kRequestCooldownSeconds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem.Position;
import org.littletonrobotics.junction.Logger;

/**
 * Pede ao intake para descer (LB) ou subir (RB). E um comando instantaneo que carrega a PROTECAO
 * CONTRA CLIQUE DUPLO: o pedido e ignorado (nada acontece) quando
 * <ul>
 *   <li>o intake ja esta na posicao pedida ou indo para ela (ex: LB duas vezes), ou
 *   <li>chega menos de {@code kRequestCooldownSeconds} depois de um pedido aceito.
 * </ul>
 * Se o pivo estiver em falha, repetir a mesma posicao e aceito (serve para tentar de novo).
 */
public class MoveIntakeCommand extends Command {
  private final IntakePivotSubsystem pivot;
  private final Position requested;

  public MoveIntakeCommand(IntakePivotSubsystem pivot, Position requested) {
    this.pivot = pivot;
    this.requested = requested;
    setName("MoveIntake " + requested.name());
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    if (requested == pivot.getTarget() && !pivot.isFaulted()) {
      Logger.recordOutput(
          "Intake/Pivot/LastRequest", requested.name() + " ignorado: ja esta indo/em " + requested.name());
      return;
    }
    if (pivot.getSecondsSinceTargetChange() < kRequestCooldownSeconds) {
      Logger.recordOutput(
          "Intake/Pivot/LastRequest", requested.name() + " ignorado: clique duplo (espera)");
      return;
    }

    pivot.setTarget(requested);
    Logger.recordOutput("Intake/Pivot/LastRequest", requested.name() + " aceito");
  }

  @Override
  public boolean isFinished() {
    return true; // instantaneo: o subsistema continua controlando a posicao sozinho
  }
}
