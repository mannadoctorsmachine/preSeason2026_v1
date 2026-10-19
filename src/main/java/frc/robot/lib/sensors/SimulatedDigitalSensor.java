package frc.robot.lib.sensors;

/** Implementacao simulada de {@link DigitalSensor}; o estado e definido manualmente (ou por teste). */
public class SimulatedDigitalSensor implements DigitalSensor {
  private boolean triggered = false;

  public void setTriggered(boolean triggered) {
    this.triggered = triggered;
  }

  @Override
  public boolean isTriggered() {
    return triggered;
  }

  @Override
  public void updateInputs(DigitalSensorInputs inputs) {
    inputs.rawValue = triggered;
    inputs.triggered = triggered;
  }

  @Override
  public Object getNativeSensor() {
    return null;
  }
}
