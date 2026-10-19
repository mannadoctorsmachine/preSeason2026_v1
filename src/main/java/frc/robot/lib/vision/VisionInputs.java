package frc.robot.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Telemetria bruta de uma camera de visao (padrao IO layer do AdvantageKit). */
public class VisionInputs implements LoggableInputs {
  /** Camera enviando dados (heartbeat ativo). */
  public boolean connected = false;

  /** Ha uma estimativa de pose nova e valida neste ciclo (antes dos filtros de qualidade). */
  public boolean hasEstimate = false;

  /** Pose do robo no campo (origem no lado azul). */
  public Pose2d pose = Pose2d.kZero;

  /** Instante da captura, na base de tempo do FPGA (segundos). */
  public double timestampSeconds = 0.0;

  /** Numero de AprilTags usadas na estimativa. */
  public int tagCount = 0;

  /** Distancia media (m) ate as tags usadas. */
  public double avgTagDistanceMeters = 0.0;

  /** Area media das tags na imagem (% da imagem). */
  public double avgTagArea = 0.0;

  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("HasEstimate", hasEstimate);
    table.put("PoseX", pose.getX());
    table.put("PoseY", pose.getY());
    table.put("PoseRotationRad", pose.getRotation().getRadians());
    table.put("TimestampSeconds", timestampSeconds);
    table.put("TagCount", tagCount);
    table.put("AvgTagDistanceMeters", avgTagDistanceMeters);
    table.put("AvgTagArea", avgTagArea);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    hasEstimate = table.get("HasEstimate", hasEstimate);
    pose =
        new Pose2d(
            table.get("PoseX", pose.getX()),
            table.get("PoseY", pose.getY()),
            new Rotation2d(table.get("PoseRotationRad", pose.getRotation().getRadians())));
    timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
    tagCount = table.get("TagCount", tagCount);
    avgTagDistanceMeters = table.get("AvgTagDistanceMeters", avgTagDistanceMeters);
    avgTagArea = table.get("AvgTagArea", avgTagArea);
  }
}
