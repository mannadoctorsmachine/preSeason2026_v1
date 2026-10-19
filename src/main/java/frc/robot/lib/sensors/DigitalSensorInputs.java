package frc.robot.lib.sensors;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Telemetria de um sensor digital (beam break, limit switch, sensor hall/indutivo). */
public class DigitalSensorInputs implements LoggableInputs {
  /** Nivel logico bruto lido no pino (sem inversao nem debounce). */
  public boolean rawValue = false;

  /** Estado logico final: true = sensor ATIVADO (inversao e debounce ja aplicados). */
  public boolean triggered = false;

  @Override
  public void toLog(LogTable table) {
    table.put("RawValue", rawValue);
    table.put("Triggered", triggered);
  }

  @Override
  public void fromLog(LogTable table) {
    rawValue = table.get("RawValue", rawValue);
    triggered = table.get("Triggered", triggered);
  }
}
