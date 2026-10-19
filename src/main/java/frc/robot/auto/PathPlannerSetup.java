package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.lib.navigation.AllianceUtil;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Configura o PathPlanner utilizando os dados fisicos carregados do YAGSL.
 */
public final class PathPlannerSetup {
  private PathPlannerSetup() {
    throw new UnsupportedOperationException("Classe utilitaria!");
  }

  public static void configure(SwerveSubsystem swerve, RobotState robotState) {
    Translation2d[] locations = swerve.getSwerveDrive().swerveDriveConfiguration.moduleLocationsMeters;

    ModuleConfig moduleConfig =
        new ModuleConfig(
            Units.inchesToMeters(2.0),
            DriveConstants.kMaxSpeedMps,
            1.19,
            DCMotor.getKrakenX60(1),
            6.75,
            40.0,
            1);

    RobotConfig robotConfig =
        new RobotConfig(
            62.0,
            5.0,
            moduleConfig,
            locations[0],
            locations[1],
            locations[2],
            locations[3]);

    AutoBuilder.configure(
        robotState::getPose,
        robotState::resetPose,
        swerve::getChassisSpeeds,
        (speeds, feedforwards) -> swerve.runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)),
        robotConfig,
        AllianceUtil::isRed,
        swerve);

    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("Auto/ActivePath", poses.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("Auto/TargetPose", pose));
  }
}
