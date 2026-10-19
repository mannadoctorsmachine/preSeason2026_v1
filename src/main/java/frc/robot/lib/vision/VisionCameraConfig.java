package frc.robot.lib.vision;

/**
 * Configuracao de uma camera de visao. A posicao da camera no robo (robot-to-camera) deve ser
 * configurada na interface web da propria Limelight.
 */
public class VisionCameraConfig {
  /** Nome da Limelight (hostname configurado na camera, ex: "limelight-front"). */
  public final String name;

  /**
   * Prioridade: MENOR numero = MAIS prioritaria (0 = principal). Cameras com a mesma prioridade
   * formam um grupo e contribuem juntas. Um grupo so envia dados se TODOS os grupos mais prioritarios
   * estiverem sem visao.
   */
  public int priority = 0;

  /** Usa MegaTag 2 (precisa do gyro, so estima X/Y) em vez de MegaTag 1 (estima tambem rotacao). */
  public boolean useMegaTag2 = true;

  /** Rejeita medicoes cuja distancia media as tags e maior que isto (m). */
  public double maxAvgTagDistanceMeters = 4.5;

  /** Numero minimo de tags para aceitar a medicao. */
  public int minTagCount = 1;

  /** Desvio padrao base de X/Y (m) para 1 tag a 1 m. Cresce com distancia^2 e cai com o numero de tags. */
  public double xyStdDevBase = 0.05;

  /** Desvio padrao de rotacao (rad). Alto = ignora a rotacao da visao (recomendado com MegaTag 2). */
  public double thetaStdDevRad = 1.0e6;

  /** Multiplicador de confianca da camera (maior = confia menos). */
  public double stdDevScale = 1.0;

  public boolean enabled = true;

  public VisionCameraConfig(String name) {
    this.name = name;
  }

  public VisionCameraConfig withPriority(int priority) {
    this.priority = priority;
    return this;
  }

  public VisionCameraConfig withMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
    return this;
  }

  public VisionCameraConfig withMaxTagDistance(double meters) {
    this.maxAvgTagDistanceMeters = meters;
    return this;
  }

  public VisionCameraConfig withMinTagCount(int count) {
    this.minTagCount = count;
    return this;
  }

  public VisionCameraConfig withStdDevs(double xyBase, double thetaRad) {
    this.xyStdDevBase = xyBase;
    this.thetaStdDevRad = thetaRad;
    return this;
  }

  public VisionCameraConfig withStdDevScale(double scale) {
    this.stdDevScale = scale;
    return this;
  }
}
