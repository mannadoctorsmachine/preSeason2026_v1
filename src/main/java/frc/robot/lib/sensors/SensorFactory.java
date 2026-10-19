package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Fabrica para instanciacao facil e segura de sensores.
 * Em simulacao, tipos de hardware sao substituidos automaticamente pela versao simulada,
 * entao o codigo dos subsistemas nunca precisa saber qual sensor esta por baixo.
 */
public final class SensorFactory {

  public enum GyroType {
    PIGEON2,
    NAVX,
    SIMULATED
  }

  public enum AbsoluteEncoderType {
    CANCODER,
    /** REV Through Bore ou Thrifty Absolute Magnetic ligado a uma porta DIO. */
    DUTY_CYCLE,
    SIMULATED
  }

  public enum DistanceSensorType {
    CANRANGE,
    SIMULATED
  }

  private SensorFactory() {
    throw new UnsupportedOperationException("Classe utilitaria de fabrica!");
  }

  /**
   * Cria um giroscopio.
   * @param type Tipo de hardware.
   * @param canId ID CAN (Pigeon 2). Ignorado para NavX.
   * @param canBus Barramento CAN ("" para o RIO). Ignorado para NavX.
   */
  public static Gyro createGyro(GyroType type, int canId, String canBus) {
    if (RobotBase.isSimulation()) {
      return new SimulatedGyro();
    }
    return switch (type) {
      case PIGEON2 -> new Pigeon2Gyro(canId, canBus);
      case NAVX -> new NavXGyro();
      case SIMULATED -> new SimulatedGyro();
    };
  }

  /**
   * Cria um encoder absoluto.
   * @param type Tipo de hardware.
   * @param id ID CAN (CANCODER) ou canal DIO (DUTY_CYCLE).
   * @param canBus Barramento CAN ("" para o RIO). Ignorado para DUTY_CYCLE.
   * @param config Offset, inversao e faixa.
   */
  public static AbsoluteEncoder createAbsoluteEncoder(
      AbsoluteEncoderType type, int id, String canBus, AbsoluteEncoderConfig config) {
    if (RobotBase.isSimulation()) {
      return new SimulatedAbsoluteEncoder(config);
    }
    return switch (type) {
      case CANCODER -> new CANcoderEncoder(id, canBus, config);
      case DUTY_CYCLE -> new DutyCycleAbsoluteEncoder(id, config);
      case SIMULATED -> new SimulatedAbsoluteEncoder(config);
    };
  }

  /**
   * Cria um sensor digital (beam break / limit switch) em uma porta DIO.
   * @param dioChannel Canal DIO.
   * @param inverted true se o sensor ativa em nivel LOW (comum em beam breaks e chaves ao GND).
   * @param debounceSeconds Debounce em segundos (0 = desligado).
   */
  public static DigitalSensor createDigital(int dioChannel, boolean inverted, double debounceSeconds) {
    if (RobotBase.isSimulation()) {
      return new SimulatedDigitalSensor();
    }
    return new DigitalInputSensor(dioChannel, inverted, debounceSeconds);
  }

  /** Cria um sensor de distancia time-of-flight. */
  public static DistanceSensor createDistance(
      DistanceSensorType type, int canId, String canBus, DistanceSensorConfig config) {
    if (RobotBase.isSimulation()) {
      return new SimulatedDistanceSensor(config);
    }
    return switch (type) {
      case CANRANGE -> new CANrangeSensor(canId, canBus, config);
      case SIMULATED -> new SimulatedDistanceSensor(config);
    };
  }
}
