package frc.robot.lib.motors;

/**
 * Estrutura de dados contendo os dados de telemetria e estado lidos do motor.
 * Segue o padrao IO layer do AdvantageKit/WPILib para logging e determinismo.
 */
public class MotorInputs {
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
}
