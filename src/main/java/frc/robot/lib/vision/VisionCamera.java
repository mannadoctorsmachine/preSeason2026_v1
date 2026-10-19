package frc.robot.lib.vision;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface unificada de uma camera de visao com AprilTags (Limelight e Simulada).
 * O resto do codigo nunca importa o LimelightHelpers: so conhece esta interface.
 */
public interface VisionCamera {

  /** @return Configuracao (nome, prioridade, filtros) desta camera. */
  VisionCameraConfig getConfig();

  /**
   * Informa a orientacao do robo (gyro) para a camera. Obrigatorio para MegaTag 2.
   * @param yaw Heading do robo no campo.
   * @param yawRateRadPerSec Velocidade angular do robo.
   */
  void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec);

  /**
   * Atualiza a estrutura de telemetria para logging e determinismo de simulacao/replay.
   * @param inputs Objeto de telemetria a ser preenchido.
   */
  void updateInputs(VisionInputs inputs);

  /** Restringe quais tags a camera usa (null ou vazio = todas). */
  default void setTagFilter(int[] tagIds) {}

  /** Troca o pipeline da camera. */
  default void setPipeline(int pipeline) {}
}
