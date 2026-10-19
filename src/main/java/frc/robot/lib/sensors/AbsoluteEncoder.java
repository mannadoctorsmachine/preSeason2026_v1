package frc.robot.lib.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface unificada para encoders absolutos (CANcoder, Through Bore / Thrifty via DIO e Simulado).
 * Uso tipico: zerar o estercamento do swerve, bracos e pivots na inicializacao.
 */
public interface AbsoluteEncoder {

  /** @return Posicao absoluta em rotacoes (offset e inversao ja aplicados). */
  double getPosition();

  /** @return Posicao absoluta em radianos. */
  default double getPositionRadians() {
    return getPosition() * 2.0 * Math.PI;
  }

  /** @return Posicao absoluta como Rotation2d. */
  default Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(getPosition());
  }

  /** @return Velocidade em rotacoes por segundo. */
  double getVelocity();

  /** @return true se o encoder esta conectado e com leitura valida. */
  boolean isConnected();

  /**
   * Atualiza a estrutura de telemetria para logging e determinismo de simulacao/replay.
   * @param inputs Objeto de telemetria a ser preenchido.
   */
  void updateInputs(AbsoluteEncoderInputs inputs);

  /** @return Objeto nativo do fabricante (CANcoder, DutyCycleEncoder) ou null. */
  Object getNativeSensor();
}
