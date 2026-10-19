package frc.robot.lib.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

/**
 * Implementacao de {@link DistanceSensor} para o CTRE CANrange (Phoenix 6).
 * A deteccao por limiar e feita no proprio hardware (ProximityParams).
 */
public class CANrangeSensor implements DistanceSensor {
  private final CANrange canrange;
  private final StatusSignal<Distance> distanceSignal;
  private final StatusSignal<Boolean> detectedSignal;
  private final StatusSignal<Double> signalStrengthSignal;
  private boolean connected = false;

  public CANrangeSensor(int canId, String canBus, DistanceSensorConfig config) {
    this.canrange =
        (canBus == null || canBus.isEmpty())
            ? new CANrange(canId)
            : new CANrange(canId, new CANBus(canBus));

    CANrangeConfiguration cfg = new CANrangeConfiguration();
    cfg.ProximityParams.ProximityThreshold = config.detectionThresholdMeters;
    cfg.ProximityParams.ProximityHysteresis = config.hysteresisMeters;
    cfg.FovParams.FOVRangeX = config.fovDegrees;
    cfg.FovParams.FOVRangeY = config.fovDegrees;
    canrange.getConfigurator().apply(cfg);

    distanceSignal = canrange.getDistance();
    detectedSignal = canrange.getIsDetected();
    signalStrengthSignal = canrange.getSignalStrength();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, distanceSignal, detectedSignal, signalStrengthSignal);
    canrange.optimizeBusUtilization();
  }

  public CANrangeSensor(int canId, DistanceSensorConfig config) {
    this(canId, "", config);
  }

  private void refresh() {
    connected =
        BaseStatusSignal.refreshAll(distanceSignal, detectedSignal, signalStrengthSignal).isOK();
  }

  @Override
  public double getDistanceMeters() {
    return distanceSignal.getValueAsDouble();
  }

  @Override
  public boolean isDetected() {
    return detectedSignal.getValue();
  }

  @Override
  public double getSignalStrength() {
    return signalStrengthSignal.getValueAsDouble();
  }

  @Override
  public boolean isConnected() {
    return connected;
  }

  @Override
  public void updateInputs(DistanceSensorInputs inputs) {
    refresh();
    inputs.connected = connected;
    inputs.distanceMeters = distanceSignal.getValueAsDouble();
    inputs.detected = detectedSignal.getValue();
    inputs.signalStrength = signalStrengthSignal.getValueAsDouble();
  }

  @Override
  public Object getNativeSensor() {
    return canrange;
  }
}
