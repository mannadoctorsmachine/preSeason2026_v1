package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.navigation.AllianceUtil;
import java.io.File;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Subsistema do Swerve utilizando a biblioteca YAGSL (Yet Another Generic Swerve Library).
 * Todas as configuracoes dos modulos (IDs dos motores Kraken X60 e Spark MAX Neo,
 * encoders CANcoder, relacoes de engrenagem e offsets calibrados) sao lidas diretamente
 * dos arquivos JSON em 'src/main/deploy/swerve'.
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  public SwerveSubsystem() {
    this(DriveConstants.kMaxSpeedMps);
  }

  public SwerveSubsystem(double maxSpeedMps) {
    try {
      File swerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
      this.swerveDrive = new SwerveParser(swerveDirectory).createSwerveDrive(maxSpeedMps);
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    } catch (Exception e) {
      throw new RuntimeException("Falha ao inicializar o Swerve a partir de deploy/swerve: " + e.getMessage(), e);
    }
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();

    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/HeadingDeg", getHeading().getDegrees());
    Logger.recordOutput("Swerve/MeasuredSpeeds", getChassisSpeeds());
    Logger.recordOutput("Swerve/MeasuredStates", getModuleStates());
  }

  // ---------------------------------------------------------------------------------------------
  // Comandos de movimento
  // ---------------------------------------------------------------------------------------------

  /** Move o robo com velocidades relativas ao chassi (Robot-Oriented). */
  public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {
    swerveDrive.drive(robotRelativeSpeeds);
  }

  /** Move o robo com velocidades relativas ao campo (Field-Oriented). */
  public void runFieldVelocity(ChassisSpeeds fieldRelativeSpeeds) {
    swerveDrive.driveFieldOriented(fieldRelativeSpeeds);
  }

  /** Para o robo completamente. */
  public void stop() {
    swerveDrive.drive(new ChassisSpeeds());
  }

  /** Trava as rodas em formato de 'X' (impede ser empurrado na defesa). */
  public void lockWheels() {
    swerveDrive.lockPose();
  }

  /** Zera o giroscopio (redefine a frente atual como 0 graus do campo). */
  public void zeroHeading() {
    swerveDrive.zeroGyro();
  }

  /**
   * Comando de teleoperacao via joystick (Field-Oriented) com deadband e curva quadratica.
   */
  public Command driveCommand(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return run(
        () -> {
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.08);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.08);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), 0.08);

          // Curva quadratica para precisao em baixa velocidade
          x = Math.copySign(x * x, x) * swerveDrive.getMaximumChassisVelocity();
          y = Math.copySign(y * y, y) * swerveDrive.getMaximumChassisVelocity();
          omega = Math.copySign(omega * omega, omega) * swerveDrive.getMaximumChassisAngularVelocity();

          double allianceSign = AllianceUtil.isRed() ? -1.0 : 1.0;
          runFieldVelocity(new ChassisSpeeds(allianceSign * x, allianceSign * y, omega));
        }).withName("SwerveDriveCommand");
  }

  // ---------------------------------------------------------------------------------------------
  // Getters e Leituras
  // ---------------------------------------------------------------------------------------------

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public SwerveModuleState[] getModuleStates() {
    return swerveDrive.getStates();
  }

  public SwerveModulePosition[] getModulePositions() {
    return swerveDrive.getModulePositions();
  }

  public double getCircumradiusMeters() {
    Translation2d[] locs = swerveDrive.swerveDriveConfiguration.moduleLocationsMeters;
    if (locs != null && locs.length > 0) {
      return locs[0].getNorm() + 0.10;
    }
    return 0.45;
  }
}
