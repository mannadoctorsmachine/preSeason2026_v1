package frc.robot.lib.sensors;

/**
 * Implementacao simulada de {@link AbsoluteEncoder}.
 * A simulacao do mecanismo (ex: posicao do SimulatedMotor) deve informar a posicao via
 * {@link #setSimulatedPosition(double)}.
 */
public class SimulatedAbsoluteEncoder implements AbsoluteEncoder {
  private final AbsoluteEncoderConfig config;
  private double positionRotations = 0.0;
  private double velocityRotPerSec = 0.0;

  public SimulatedAbsoluteEncoder(AbsoluteEncoderConfig config) {
    this.config = config;
  }

  /** Define a posicao real do mecanismo (rotacoes); o wrap da config e aplicado na leitura. */
  public void setSimulatedPosition(double rotations) {
    this.positionRotations = rotations;
  }

  public void setSimulatedVelocity(double rotationsPerSec) {
    this.velocityRotPerSec = rotationsPerSec;
  }

  @Override
  public double getPosition() {
    return config.wrap(positionRotations);
  }

  @Override
  public double getVelocity() {
    return velocityRotPerSec;
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void updateInputs(AbsoluteEncoderInputs inputs) {
    inputs.connected = true;
    inputs.positionRotations = getPosition();
    inputs.positionRadians = inputs.positionRotations * 2.0 * Math.PI;
    inputs.velocityRotationsPerSec = velocityRotPerSec;
  }

  @Override
  public Object getNativeSensor() {
    return null;
  }
}
