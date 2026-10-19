package frc.robot.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Camera simulada: por padrao nao ve nada. Use {@link #setSimulatedMeasurement} para injetar uma
 * leitura (ex: em testes de prioridade/fallback) e {@link #clearMeasurement()} para "perder" as tags.
 */
public class SimulatedVisionCamera implements VisionCamera {
  private final VisionCameraConfig config;
  private Pose2d pose = null;
  private int tagCount = 0;
  private double avgDistance = 2.0;

  public SimulatedVisionCamera(VisionCameraConfig config) {
    this.config = config;
  }

  public void setSimulatedMeasurement(Pose2d pose, int tagCount, double avgTagDistanceMeters) {
    this.pose = pose;
    this.tagCount = tagCount;
    this.avgDistance = avgTagDistanceMeters;
  }

  public void clearMeasurement() {
    this.pose = null;
    this.tagCount = 0;
  }

  @Override
  public VisionCameraConfig getConfig() {
    return config;
  }

  @Override
  public void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec) {}

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.connected = true;
    inputs.hasEstimate = pose != null && tagCount > 0;
    inputs.tagCount = tagCount;
    if (inputs.hasEstimate) {
      inputs.pose = pose;
      inputs.timestampSeconds = Timer.getFPGATimestamp();
      inputs.avgTagDistanceMeters = avgDistance;
      inputs.avgTagArea = 1.0;
    }
  }
}
