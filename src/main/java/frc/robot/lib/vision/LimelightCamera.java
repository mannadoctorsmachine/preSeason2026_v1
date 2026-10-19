package frc.robot.lib.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;

/**
 * Implementacao de {@link VisionCamera} para a Limelight. E a UNICA classe do projeto que usa o
 * {@link LimelightHelpers}; se a API dele mudar, so este arquivo precisa ser ajustado.
 *
 * <p>Requer o arquivo LimelightHelpers.java em src/main/java/frc/robot/ (baixar da documentacao
 * oficial da Limelight).
 */
public class LimelightCamera implements VisionCamera {
  private static final double kHeartbeatTimeoutSeconds = 0.5;

  private final VisionCameraConfig config;

  private double lastHeartbeat = -1.0;
  private double lastHeartbeatChange = Double.NEGATIVE_INFINITY;
  private double lastTimestamp = -1.0;

  public LimelightCamera(VisionCameraConfig config) {
    this.config = config;
  }

  @Override
  public VisionCameraConfig getConfig() {
    return config;
  }

  @Override
  public void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec) {
    LimelightHelpers.SetRobotOrientation(
        config.name, yaw.getDegrees(), Math.toDegrees(yawRateRadPerSec), 0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public void setTagFilter(int[] tagIds) {
    LimelightHelpers.SetFiducialIDFiltersOverride(config.name, tagIds == null ? new int[0] : tagIds);
  }

  @Override
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(config.name, pipeline);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    double now = Timer.getFPGATimestamp();

    // Heartbeat: o valor "hb" incrementa continuamente enquanto a camera esta viva
    double hb =
        NetworkTableInstance.getDefault().getTable(config.name).getEntry("hb").getDouble(-1.0);
    if (hb != lastHeartbeat) {
      lastHeartbeat = hb;
      lastHeartbeatChange = now;
    }
    inputs.connected = hb >= 0.0 && (now - lastHeartbeatChange) < kHeartbeatTimeoutSeconds;

    inputs.hasEstimate = false;
    if (!inputs.connected) {
      inputs.tagCount = 0;
      return;
    }

    LimelightHelpers.PoseEstimate estimate =
        config.useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);

    if (estimate == null || estimate.tagCount == 0) {
      inputs.tagCount = 0;
      return;
    }

    // Descarta repeticoes da mesma captura (a camera atualiza menos que 50 Hz)
    if (estimate.timestampSeconds == lastTimestamp) {
      inputs.tagCount = estimate.tagCount;
      return;
    }
    lastTimestamp = estimate.timestampSeconds;

    inputs.hasEstimate = true;
    inputs.pose = estimate.pose;
    inputs.timestampSeconds = estimate.timestampSeconds;
    inputs.tagCount = estimate.tagCount;
    inputs.avgTagDistanceMeters = estimate.avgTagDist;
    inputs.avgTagArea = estimate.avgTagArea;
  }
}
