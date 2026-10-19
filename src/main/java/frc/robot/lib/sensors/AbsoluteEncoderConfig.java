package frc.robot.lib.sensors;

/**
 * Configuracao de um encoder absoluto (CANcoder, REV Through Bore, Thrifty Absolute Magnetic).
 * Todas as posicoes sao em rotacoes (1.0 = 360 graus) no eixo onde o encoder esta montado.
 */
public class AbsoluteEncoderConfig {
  /**
   * Offset do ima: posicao absoluta lida (em rotacoes) quando o mecanismo esta na posicao zero.
   * O valor reportado sera (leitura - offset).
   */
  public double offsetRotations = 0.0;

  /** Se true, inverte o sentido positivo de leitura. */
  public boolean inverted = false;

  /** Menor valor reportado (rotacoes). -0.5 => faixa [-0.5, 0.5); 0.0 => faixa [0, 1). */
  public double minRotations = -0.5;

  public AbsoluteEncoderConfig() {}

  public AbsoluteEncoderConfig withOffset(double rotations) {
    this.offsetRotations = rotations;
    return this;
  }

  public AbsoluteEncoderConfig withInverted(boolean inverted) {
    this.inverted = inverted;
    return this;
  }

  public AbsoluteEncoderConfig withRange(double minRotations) {
    this.minRotations = minRotations;
    return this;
  }

  /** Aplica wrap em [minRotations, minRotations + 1). */
  double wrap(double rotations) {
    double r = (rotations - minRotations) % 1.0;
    if (r < 0) {
      r += 1.0;
    }
    return r + minRotations;
  }
}
