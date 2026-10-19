package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interface unificada para sensores digitais: beam break (infravermelho), limit switch, sensor Hall.
 * "Triggered" significa que o sensor esta ATIVADO (objeto presente / chave pressionada),
 * independente da logica eletrica (normalmente aberto ou fechado).
 */
public interface DigitalSensor {

  /** @return true se o sensor esta ativado (inversao e debounce aplicados). */
  boolean isTriggered();

  /**
   * Atualiza a estrutura de telemetria para logging e determinismo de simulacao/replay.
   * @param inputs Objeto de telemetria a ser preenchido.
   */
  void updateInputs(DigitalSensorInputs inputs);

  /** @return Trigger do command-based para uso direto em bindings e command groups. */
  default Trigger asTrigger() {
    return new Trigger(this::isTriggered);
  }

  /** @return Objeto nativo (DigitalInput) ou null. */
  Object getNativeSensor();
}
