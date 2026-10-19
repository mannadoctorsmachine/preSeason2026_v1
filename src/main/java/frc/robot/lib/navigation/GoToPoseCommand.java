package frc.robot.lib.navigation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Vai ate uma coordenada do campo a partir de QUALQUER posicao atual, desviando das zonas
 * restritas cadastradas em {@link ObstacleField}.
 *
 * <ul>
 *   <li>A rota e recalculada a cada ciclo a partir da pose atual (Liang-Barsky + grafo de
 *       visibilidade), entao o robo se recupera de empurroes e desvios.
 *   <li>A velocidade segue um perfil trapezoidal (ProfiledPID) sobre a DISTANCIA RESTANTE da rota,
 *       garantindo parada suave exatamente no destino.
 *   <li>A orientacao do chassi fica FIXA: mantem o heading de quando o comando iniciou, ou gira
 *       suavemente para o heading do destino se um Pose2d for informado.
 * </ul>
 */
public class GoToPoseCommand extends Command {
  private final SwerveSubsystem swerve;
  private final ObstacleField fieldBlue;
  private final Translation2d goalBlue;
  private final Rotation2d fixedHeadingBlue; // null = manter heading do inicio
  private final GoToConfig config;

  // Valores efetivos (ja espelhados para a alianca atual), definidos em initialize()
  private ObstacleField field;
  private Translation2d goal;
  private Rotation2d fixedHeading;

  private final ProfiledPIDController distanceController;
  private final ProfiledPIDController headingController;

  private Rotation2d targetHeading = Rotation2d.kZero;
  private List<Translation2d> currentPath = List.of();

  /** Vai ate {@code goal} mantendo o heading que o robo tinha ao iniciar o comando. */
  public GoToPoseCommand(SwerveSubsystem swerve, ObstacleField field, Translation2d goal) {
    this(swerve, field, goal, null, new GoToConfig());
  }

  /** Vai ate a pose {@code goal}, girando suavemente para o heading do destino. */
  public GoToPoseCommand(SwerveSubsystem swerve, ObstacleField field, Pose2d goal) {
    this(swerve, field, goal.getTranslation(), goal.getRotation(), new GoToConfig());
  }

  public GoToPoseCommand(
      SwerveSubsystem swerve,
      ObstacleField field,
      Translation2d goal,
      Rotation2d fixedHeading,
      GoToConfig config) {
    this.swerve = swerve;
    this.fieldBlue = field;
    this.goalBlue = goal;
    this.fixedHeadingBlue = fixedHeading;
    this.config = config;
    this.field = field;
    this.goal = goal;
    this.fixedHeading = fixedHeading;

    distanceController =
        new ProfiledPIDController(
            config.distanceKp,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(config.maxVelocityMps, config.maxAccelerationMps2));
    headingController =
        new ProfiledPIDController(
            config.headingKp,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                config.maxAngularVelocityRadPerSec, config.maxAngularAccelerationRadPerSec2));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    if (config.flipForAlliance) {
      field = fieldBlue.forCurrentAlliance();
      goal = AllianceUtil.flipIfRed(goalBlue);
      fixedHeading =
          (fixedHeadingBlue != null) ? AllianceUtil.flipIfRed(fixedHeadingBlue) : null;
    }

    Pose2d pose = swerve.getPose();
    targetHeading = (fixedHeading != null) ? fixedHeading : pose.getRotation();
    Logger.recordOutput("GoTo/Goal", new Pose2d(goal, targetHeading));

    currentPath = computePath(pose.getTranslation());
    double remaining = ObstacleField.pathLength(pose.getTranslation(), currentPath);

    // Velocidade inicial (m/s) na direcao do proximo ponto, para nao dar tranco se ja estiver andando
    Translation2d toNext = currentPath.get(0).minus(pose.getTranslation());
    double speedToward = 0.0;
    if (toNext.getNorm() > 1e-6) {
      ChassisSpeeds v = swerve.getFieldVelocity();
      Translation2d dir = toNext.div(toNext.getNorm());
      speedToward = Math.max(0.0, v.vxMetersPerSecond * dir.getX() + v.vyMetersPerSecond * dir.getY());
    }
    distanceController.reset(remaining, -speedToward);
    headingController.reset(
        pose.getRotation().getRadians(), swerve.getChassisSpeeds().omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    Pose2d pose = swerve.getPose();
    Translation2d position = pose.getTranslation();

    currentPath = computePath(position);
    double remaining = ObstacleField.pathLength(position, currentPath);

    Translation2d next = currentPath.get(0);
    Translation2d toNext = next.minus(position);
    double norm = toNext.getNorm();

    // Velocidade escalar sobre a rota (o setpoint do perfil desce de "remaining" ate 0)
    double output = distanceController.calculate(remaining, 0.0);
    double speed = -(output + distanceController.getSetpoint().velocity);
    speed = Math.max(0.0, Math.min(config.maxVelocityMps, speed));

    double vx = 0.0;
    double vy = 0.0;
    if (norm > 1e-6) {
      vx = toNext.getX() / norm * speed;
      vy = toNext.getY() / norm * speed;
    }

    double omega =
        headingController.calculate(pose.getRotation().getRadians(), targetHeading.getRadians())
            + headingController.getSetpoint().velocity;

    swerve.runFieldVelocity(new ChassisSpeeds(vx, vy, omega));

    Translation2d[] logged = new Translation2d[currentPath.size() + 1];
    logged[0] = position;
    for (int i = 0; i < currentPath.size(); i++) {
      logged[i + 1] = currentPath.get(i);
    }
    Logger.recordOutput("GoTo/Path", logged);
    Logger.recordOutput("GoTo/RemainingMeters", remaining);
  }

  /** Rota (sem o ponto de partida, terminando no destino) com pontos ja atingidos descartados. */
  private List<Translation2d> computePath(Translation2d position) {
    List<Translation2d> path =
        field
            .planPath(position, goal)
            .orElseGet(
                () -> {
                  DriverStation.reportWarning(
                      "GoToPoseCommand: sem rota livre ate o destino " + goal + "; indo em linha reta.",
                      false);
                  return List.of(goal);
                });

    List<Translation2d> trimmed = new ArrayList<>(path);
    while (trimmed.size() > 1
        && position.getDistance(trimmed.get(0)) < config.waypointToleranceMeters) {
      trimmed.remove(0);
    }
    return trimmed;
  }

  @Override
  public boolean isFinished() {
    Pose2d pose = swerve.getPose();
    ChassisSpeeds v = swerve.getFieldVelocity();
    double headingError =
        Math.abs(pose.getRotation().minus(targetHeading).getRadians());
    double speed = Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond);

    return pose.getTranslation().getDistance(goal) < config.positionToleranceMeters
        && headingError < config.headingToleranceRad
        && speed < config.velocityToleranceMps;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  /** @return Rota atual (para desenhar no dashboard/Field2d). */
  public List<Translation2d> getCurrentPath() {
    return List.copyOf(currentPath);
  }
}
