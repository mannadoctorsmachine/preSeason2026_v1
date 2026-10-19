package frc.robot.lib.sensors;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Implementacao de {@link DigitalSensor} para uma porta DIO do RoboRIO.
 *
 * @param inverted Se true, o sensor e considerado ativado quando o pino le LOW
 *     (tipico de beam breaks e limit switches ligados ao GND).
 * @param debounceSeconds Tempo de debounce (0 = sem debounce).
 */
public class DigitalInputSensor implements DigitalSensor {
  private final DigitalInput input;
  private final boolean inverted;
  private final Debouncer debouncer;

  public DigitalInputSensor(int dioChannel, boolean inverted, double debounceSeconds) {
    this.input = new DigitalInput(dioChannel);
    this.inverted = inverted;
    this.debouncer =
        (debounceSeconds > 0) ? new Debouncer(debounceSeconds, Debouncer.DebounceType.kBoth) : null;
  }

  private boolean raw() {
    return input.get();
  }

  private boolean logical() {
    boolean value = raw() ^ inverted;
    return (debouncer != null) ? debouncer.calculate(value) : value;
  }

  @Override
  public boolean isTriggered() {
    return logical();
  }

  @Override
  public void updateInputs(DigitalSensorInputs inputs) {
    inputs.rawValue = raw();
    inputs.triggered = logical();
  }

  @Override
  public Object getNativeSensor() {
    return input;
  }
}
