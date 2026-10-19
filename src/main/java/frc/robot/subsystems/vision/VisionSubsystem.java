package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.lib.navigation.ObstacleField;
import frc.robot.lib.vision.LimelightCamera;
import frc.robot.lib.vision.SimulatedVisionCamera;
import frc.robot.lib.vision.VisionCamera;
import frc.robot.lib.vision.VisionCameraConfig;
import frc.robot.lib.vision.VisionManager;
import frc.robot.lib.vision.VisionMeasurement;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Subsistema de visao: so conhece as cameras (via {@link VisionManager}) e o {@link RobotState}.
 * Nao conhece o swerve: a fusao com a odometria acontece dentro do RobotState.
 *
 * <p>Para adicionar uma camera, use {@link #addLimelight(VisionCameraConfig)}.
 */
public class VisionSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final VisionManager manager;

  public VisionSubsystem(RobotState robotState) {
    this.robotState = robotState;
    this.manager =
        new VisionManager(ObstacleField.kDefaultFieldLength, ObstacleField.kDefaultFieldWidth);
  }

  /** Adiciona uma Limelight (camera simulada em simulacao). @return o proprio subsistema. */
  public VisionSubsystem addLimelight(VisionCameraConfig config) {
    VisionCamera camera =
        RobotBase.isSimulation() ? new SimulatedVisionCamera(config) : new LimelightCamera(config);
    manager.addCamera(camera);
    return this;
  }

  /** Acesso ao gerenciador (modo de prioridade, timeouts, habilitar/desabilitar cameras). */
  public VisionManager getManager() {
    return manager;
  }

  @Override
  public void periodic() {
    var yaw = robotState.getHeading();
    double yawRate = robotState.getRobotRelativeVelocity().omegaRadiansPerSecond;

    List<VisionMeasurement> accepted = manager.update(yaw, yawRate);
    Pose2d[] acceptedPoses = new Pose2d[accepted.size()];
    for (int i = 0; i < accepted.size(); i++) {
      VisionMeasurement m = accepted.get(i);
      robotState.addVisionMeasurement(m.pose(), m.timestampSeconds(), m.stdDevs());
      acceptedPoses[i] = m.pose();
    }

    Logger.recordOutput("Vision/AcceptedPoses", acceptedPoses);
    Logger.recordOutput("Vision/AnyVision", manager.hasAnyVision());
    Logger.recordOutput(
        "Vision/ActiveCameras", manager.getActiveCameraNames().toArray(new String[0]));
    for (VisionManager.CameraStatus s : manager.getStatuses()) {
      String base = "Vision/" + s.name + "/";
      Logger.recordOutput(base + "HasVision", s.hasVision);
      Logger.recordOutput(base + "Active", s.active);
      Logger.recordOutput(base + "Status", s.rejectReason);
    }
  }
}
