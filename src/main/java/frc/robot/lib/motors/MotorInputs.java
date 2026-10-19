package frc.robot.lib.motors;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Estrutura de dados contendo os dados de telemetria e estado lidos do motor.
 * Segue o padrao IO layer do AdvantageKit: e registrada com {@code Logger.processInputs(...)} e
 * permite replay deterministico dos logs.
 */
public class MotorInputs implements LoggableInputs {
  /** Posicao medida no eixo de saida do mecanismo (em rotacoes). */
  public double positionRotations = 0.0;

  /** Posicao medida no eixo de saida do mecanismo (em radianos). */
  public double positionRadians = 0.0;

  /** Velocidade medida no eixo de saida do mecanismo (em rotacoes por segundo). */
  public double velocityRotationsPerSec = 0.0;

  /** Velocidade medida no eixo de saida do mecanismo (em radianos por segundo). */
  public double velocityRadiansPerSec = 0.0;

  /** Tensao eletrica aplicada aos terminais do motor (em Volts). */
  public double appliedVolts = 0.0;

  /** Corrente no estator do motor (em Amperes). */
  public double statorCurrentAmps = 0.0;

  /** Corrente extraida da fonte de alimentacao / PDP / PDH (em Amperes). */
  public double supplyCurrentAmps = 0.0;

  /** Temperatura interna do controlador/motor (em Graus Celsius). */
  public double tempCelsius = 0.0;

  /** Indica se o motor esta em modo freio (Brake). */
  public boolean isBraking = true;

  /** Indica se ha alguma falha ativa de hardware ou sobrecorrente. */
  public boolean faulted = false;

  @Override
  public void toLog(LogTable table) {
    table.put("PositionRotations", positionRotations);
    table.put("PositionRadians", positionRadians);
    table.put("VelocityRotationsPerSec", velocityRotationsPerSec);
    table.put("VelocityRadiansPerSec", velocityRadiansPerSec);
    table.put("AppliedVolts", appliedVolts);
    table.put("StatorCurrentAmps", statorCurrentAmps);
    table.put("SupplyCurrentAmps", supplyCurrentAmps);
    table.put("TempCelsius", tempCelsius);
    table.put("IsBraking", isBraking);
    table.put("Faulted", faulted);
  }

  @Override
  public void fromLog(LogTable table) {
    positionRotations = table.get("PositionRotations", positionRotations);
    positionRadians = table.get("PositionRadians", positionRadians);
    velocityRotationsPerSec = table.get("VelocityRotationsPerSec", velocityRotationsPerSec);
    velocityRadiansPerSec = table.get("VelocityRadiansPerSec", velocityRadiansPerSec);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
    supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
    tempCelsius = table.get("TempCelsius", tempCelsius);
    isBraking = table.get("IsBraking", isBraking);
    faulted = table.get("Faulted", faulted);
  }
}
