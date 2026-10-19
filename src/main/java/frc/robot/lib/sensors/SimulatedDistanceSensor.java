package frc.robot.lib.sensors;

/** Implementacao simulada de {@link DistanceSensor}; a distancia e definida manualmente. */
public class SimulatedDistanceSensor implements DistanceSensor {
  private final DistanceSensorConfig config;
  private double distanceMeters = Double.MAX_VALUE;
  private boolean detected = false;

  public SimulatedDistanceSensor(DistanceSensorConfig config) {
    this.config = config;
  }

  public void setSimulatedDistance(double meters) {
    this.distanceMeters = meters;
  }

  private boolean computeDetected() {
    double threshold = config.detectionThresholdMeters;
    // Histerese: ao sair da deteccao exige passar do limiar + histerese
    detected =
        detected
            ? distanceMeters <= threshold + config.hysteresisMeters
            : distanceMeters <= threshold;
    return detected;
  }

  @Override
  public double getDistanceMeters() {
    return distanceMeters;
  }

  @Override
  public boolean isDetected() {
    return computeDetected();
  }

  @Override
  public double getSignalStrength() {
    return 1.0;
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void updateInputs(DistanceSensorInputs inputs) {
    inputs.connected = true;
    inputs.distanceMeters = distanceMeters;
    inputs.detected = computeDetected();
    inputs.signalStrength = 1.0;
  }

  @Override
  public Object getNativeSensor() {
    return null;
  }
}
