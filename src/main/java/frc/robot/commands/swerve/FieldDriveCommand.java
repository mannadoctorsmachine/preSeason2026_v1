package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Modo FIELD: a "frente" do joystick e SEMPRE a frente que o robo tinha ao ser ligado (ou na ultima
 * vez que o piloto zerou a frente com o botao START), nao importa para onde o robo esteja virado
 * agora. Empurrar o analogico para frente leva o robo para essa mesma direcao mesmo depois de ele
 * girar. A frente nao depende da alianca.
 *
 * <p>Implementacao: a velocidade do piloto e girada pelo angulo do robo ({@code swerve.getHeading()},
 * o yaw do gyro; seu zero e a frente de quando o robo ligou ou de quando o START chamou
 * {@code zeroHeading()}) para virar velocidade no referencial do robo.
 */
public class FieldDriveCommand extends Command {
  private final SwerveSubsystem swerve;
  private final TeleopDriveInputs inputs;

  public FieldDriveCommand(SwerveSubsystem swerve, TeleopDriveInputs inputs) {
    this.swerve = swerve;
    this.inputs = inputs;
    setName("FieldDrive");
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    ChassisSpeeds driver = inputs.get();
    Rotation2d robotAngle = swerve.getHeading();

    swerve.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driver.vxMetersPerSecond,
            driver.vyMetersPerSecond,
            driver.omegaRadiansPerSecond,
            robotAngle));

    Logger.recordOutput("Drive/Boost", inputs.getBoost());
    Logger.recordOutput("Drive/FieldHeadingDeg", robotAngle.getDegrees());
  }
}
