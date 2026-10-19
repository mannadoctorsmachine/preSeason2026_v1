package frc.robot.lib.sensors;

/** Configuracao de um sensor de distancia (time-of-flight). */
public class DistanceSensorConfig {
  /** Distancia (m) abaixo da qual um objeto e considerado "detectado". */
  public double detectionThresholdMeters = 0.10;

  /** Histerese (m) da deteccao para evitar oscilacao na borda do limiar. */
  public double hysteresisMeters = 0.01;

  /** Campo de visao (graus) do cone de leitura (CANrange: 6.75 a 27). */
  public double fovDegrees = 27.0;

  public DistanceSensorConfig() {}

  public DistanceSensorConfig withDetectionThreshold(double meters) {
    this.detectionThresholdMeters = meters;
    return this;
  }

  public DistanceSensorConfig withHysteresis(double meters) {
    this.hysteresisMeters = meters;
    return this;
  }

  public DistanceSensorConfig withFov(double degrees) {
    this.fovDegrees = degrees;
    return this;
  }
}
