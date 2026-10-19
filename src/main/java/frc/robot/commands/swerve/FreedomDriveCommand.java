package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Modo FREEDOM: nao existe uma frente fixa. A direcao do joystick e sempre relativa ao robo: para
 * frente e para onde o robo esta apontando AGORA e muda conforme ele gira (como dirigir um carro).
 * Analogico para a esquerda leva o robo para o lado esquerdo DELE, e assim por diante.
 */
public class FreedomDriveCommand extends Command {
  private final SwerveSubsystem swerve;
  private final TeleopDriveInputs inputs;

  public FreedomDriveCommand(SwerveSubsystem swerve, TeleopDriveInputs inputs) {
    this.swerve = swerve;
    this.inputs = inputs;
    setName("FreedomDrive");
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    // O referencial do piloto ja e o do robo: nenhuma rotacao e necessaria
    ChassisSpeeds driver = inputs.get();
    swerve.runVelocity(driver);

    Logger.recordOutput("Drive/Boost", inputs.getBoost());
  }
}
