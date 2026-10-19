package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

/**
 * Implementacao de {@link AbsoluteEncoder} para encoders absolutos de saida PWM/duty-cycle
 * ligados a uma porta DIO do RoboRIO: REV Through Bore Encoder e Thrifty Absolute Magnetic Encoder.
 * Offset, inversao e wrap sao feitos em software.
 */
public class DutyCycleAbsoluteEncoder implements AbsoluteEncoder {
  private final DutyCycleEncoder encoder;
  private final AbsoluteEncoderConfig config;

  private double lastPosition = 0.0;
  private double lastTimestamp = Timer.getFPGATimestamp();
  private double velocityRotPerSec = 0.0;

  public DutyCycleAbsoluteEncoder(int dioChannel, AbsoluteEncoderConfig config) {
    this.encoder = new DutyCycleEncoder(dioChannel);
    this.config = config;
    this.lastPosition = getPosition();
  }

  @Override
  public double getPosition() {
    double raw = encoder.get(); // [0, 1)
    double signed = config.inverted ? -raw : raw;
    double offset = config.inverted ? -config.offsetRotations : config.offsetRotations;
    return config.wrap(signed - offset);
  }

  @Override
  public double getVelocity() {
    return velocityRotPerSec;
  }

  @Override
  public boolean isConnected() {
    return encoder.isConnected();
  }

  @Override
  public void updateInputs(AbsoluteEncoderInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double position = getPosition();
    double dt = now - lastTimestamp;
    if (dt > 1e-4) {
      // Diferenca com tratamento de wrap (menor caminho)
      double delta = position - lastPosition;
      delta -= Math.rint(delta);
      velocityRotPerSec = delta / dt;
      lastPosition = position;
      lastTimestamp = now;
    }

    inputs.connected = isConnected();
    inputs.positionRotations = position;
    inputs.positionRadians = position * 2.0 * Math.PI;
    inputs.velocityRotationsPerSec = velocityRotPerSec;
  }

  @Override
  public Object getNativeSensor() {
    return encoder;
  }
}
