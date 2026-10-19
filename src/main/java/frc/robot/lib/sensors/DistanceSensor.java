package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interface unificada para sensores de distancia time-of-flight (CTRE CANrange e Simulado).
 * Uso tipico: detectar game piece no intake/indexer sem depender de fiacao DIO.
 */
public interface DistanceSensor {

  /** @return Distancia medida em metros. */
  double getDistanceMeters();

  /** @return true se ha objeto dentro do limiar de deteccao configurado. */
  boolean isDetected();

  /** @return Intensidade do sinal (maior = mais confiavel). */
  double getSignalStrength();

  /** @return true se o sensor esta conectado. */
  boolean isConnected();

  /**
   * Atualiza a estrutura de telemetria para logging e determinismo de simulacao/replay.
   * @param inputs Objeto de telemetria a ser preenchido.
   */
  void updateInputs(DistanceSensorInputs inputs);

  /** @return Trigger do command-based que fica ativo enquanto ha objeto detectado. */
  default Trigger asTrigger() {
    return new Trigger(this::isDetected);
  }

  /** @return Objeto nativo (CANrange) ou null. */
  Object getNativeSensor();
}
