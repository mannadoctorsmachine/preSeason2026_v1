package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Guarda o modo de direcao atual (FIELD ou FREEDOM) e monta o comando padrao do swerve, que usa o
 * {@link FieldDriveCommand} ou o {@link FreedomDriveCommand} conforme o modo.
 *
 * <p>Trocar o modo ({@link #toggle()}) encerra o comando atual; o agendador recoloca o comando
 * padrao, que escolhe o comando do novo modo. Comandos que assumem o swerve (ex: GoTo) interrompem
 * a direcao normalmente e, ao terminar, a direcao volta no modo que estava.
 */
public class DriveModeSelector {
  /** Modos de direcao. */
  public enum Mode {
    /** Frente fixa: a frente de quando o robo foi ligado (ou zerada). */
    FIELD,
    /** Sem frente fixa: a frente e para onde o robo aponta agora. */
    FREEDOM
  }

  private Mode mode = Mode.FIELD;

  public Mode getMode() {
    return mode;
  }

  public boolean isField() {
    return mode == Mode.FIELD;
  }

  public void setMode(Mode newMode) {
    mode = newMode;
    publish();
  }

  /** Alterna entre FIELD e FREEDOM. */
  public void toggle() {
    setMode(mode == Mode.FIELD ? Mode.FREEDOM : Mode.FIELD);
  }

  /**
   * Comando padrao do swerve. Cada comando de direcao termina sozinho quando o modo deixa de ser o
   * dele, e o condicional escolhe de novo quando o comando padrao e reagendado.
   */
  public Command createDefaultCommand(SwerveSubsystem swerve, TeleopDriveInputs inputs) {
    Command field = new FieldDriveCommand(swerve, inputs).until(() -> mode != Mode.FIELD);
    Command freedom = new FreedomDriveCommand(swerve, inputs).until(() -> mode != Mode.FREEDOM);
    return Commands.either(field, freedom, this::isField);
  }

  /** Publica o modo no log e no dashboard (o piloto precisa ver em qual modo esta). */
  public void publish() {
    Logger.recordOutput("Drive/Mode", mode.name());
    SmartDashboard.putString("Modo de Direcao", mode.name());
  }
}
