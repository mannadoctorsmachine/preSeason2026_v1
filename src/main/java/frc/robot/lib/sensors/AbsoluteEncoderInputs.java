package frc.robot.lib.sensors;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Telemetria de um encoder absoluto (padrao IO layer do AdvantageKit). */
public class AbsoluteEncoderInputs implements LoggableInputs {
  /** Indica se o encoder esta respondendo / com sinal valido. */
  public boolean connected = false;

  /** Posicao absoluta ja com offset e inversao aplicados (rotacoes). */
  public double positionRotations = 0.0;

  /** Posicao absoluta (radianos). */
  public double positionRadians = 0.0;

  /** Velocidade (rotacoes por segundo). */
  public double velocityRotationsPerSec = 0.0;

  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("PositionRotations", positionRotations);
    table.put("PositionRadians", positionRadians);
    table.put("VelocityRotationsPerSec", velocityRotationsPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    positionRotations = table.get("PositionRotations", positionRotations);
    positionRadians = table.get("PositionRadians", positionRadians);
    velocityRotationsPerSec = table.get("VelocityRotationsPerSec", velocityRotationsPerSec);
  }
}
