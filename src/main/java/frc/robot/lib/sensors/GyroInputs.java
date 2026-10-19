package frc.robot.lib.sensors;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Estrutura de dados de telemetria de um giroscopio/IMU (padrao IO layer do AdvantageKit).
 * Convencao: angulos em radianos, positivo = anti-horario (CCW), como no WPILib.
 */
public class GyroInputs implements LoggableInputs {
  /** Indica se o sensor esta respondendo. */
  public boolean connected = false;

  /** Angulo de guinada (yaw) continuo, sem wrap (radianos). */
  public double yawRadians = 0.0;

  /** Inclinacao (pitch) em radianos. */
  public double pitchRadians = 0.0;

  /** Rolagem (roll) em radianos. */
  public double rollRadians = 0.0;

  /** Velocidade angular de guinada (rad/s). */
  public double yawVelocityRadPerSec = 0.0;

  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("YawRadians", yawRadians);
    table.put("PitchRadians", pitchRadians);
    table.put("RollRadians", rollRadians);
    table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    yawRadians = table.get("YawRadians", yawRadians);
    pitchRadians = table.get("PitchRadians", pitchRadians);
    rollRadians = table.get("RollRadians", rollRadians);
    yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
  }
}
