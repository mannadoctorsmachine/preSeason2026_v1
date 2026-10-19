package frc.robot.lib.sensors;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Telemetria de um sensor de distancia (time-of-flight). */
public class DistanceSensorInputs implements LoggableInputs {
  /** Indica se o sensor esta respondendo. */
  public boolean connected = false;

  /** Distancia medida (metros). */
  public double distanceMeters = 0.0;

  /** Objeto detectado dentro do limiar configurado. */
  public boolean detected = false;

  /** Intensidade do sinal retornado (maior = leitura mais confiavel). */
  public double signalStrength = 0.0;

  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("DistanceMeters", distanceMeters);
    table.put("Detected", detected);
    table.put("SignalStrength", signalStrength);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    distanceMeters = table.get("DistanceMeters", distanceMeters);
    detected = table.get("Detected", detected);
    signalStrength = table.get("SignalStrength", signalStrength);
  }
}
