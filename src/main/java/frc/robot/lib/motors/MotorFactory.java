package frc.robot.lib.motors;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Fabrica para instanciacao facil e segura de controladores de motor.
 * Permite alternar automaticamente para simulacao caso o robo esteja rodando em ambiente de simulacao
 * ou instanciar hardware real (Talon FX, Spark MAX, Spark FLEX).
 */
public final class MotorFactory {

  private MotorFactory() {
    throw new UnsupportedOperationException("Classe utilitaria de fabrica!");
  }

  /**
   * Cria um controlador de motor generico baseado no tipo especificado.
   * @param canId ID CAN do dispositivo.
   * @param canBus Nome do barramento CAN (ex: "rio" ou "canivore", vazio para RIO padrao).
   * @param type Tipo do motor (TALON_FX, SPARK_MAX, SPARK_FLEX, SIMULATED).
   * @param config Objeto de configuracao de parametros do motor.
   * @return Instancia implementando a interface unificada {@link MotorController}.
   */
  public static MotorController create(int canId, String canBus, MotorType type, MotorConfig config) {
    if (RobotBase.isSimulation() && type == MotorType.SIMULATED) {
      return new SimulatedMotor("Motor_CAN_" + canId, config);
    }

    return switch (type) {
      case TALON_FX -> new TalonFXMotor(canId, canBus, config);
      case SPARK_MAX, SPARK_FLEX -> new SparkMotor(canId, type, config);
      case SIMULATED -> new SimulatedMotor("Motor_CAN_" + canId, config);
    };
  }

  /**
   * Cria o motor do tipo real informado e, em simulacao, troca automaticamente por um motor
   * simulado. E o metodo recomendado para subsistemas: o mesmo codigo roda no robo e no simulador.
   * @param realType Tipo do hardware real (TALON_FX, SPARK_MAX, SPARK_FLEX).
   */
  public static MotorController createOrSim(
      int canId, String canBus, MotorType realType, MotorConfig config) {
    if (RobotBase.isSimulation()) {
      // O motor simulado usa o modelo fisico do motor real (ex: Vortex 6784 RPM livres)
      return new SimulatedMotor("Motor_CAN_" + canId, simulationModelFor(realType), config);
    }
    return create(canId, canBus, realType, config);
  }

  /** Modelo fisico (DCMotor) usado na simulacao para cada tipo de controlador. */
  private static DCMotor simulationModelFor(MotorType type) {
    return switch (type) {
      case SPARK_FLEX -> DCMotor.getNeoVortex(1);
      case SPARK_MAX -> DCMotor.getNEO(1);
      case TALON_FX, SIMULATED -> DCMotor.getKrakenX60(1);
    };
  }

  /**
   * Cria um Talon FX (Kraken X60 ou Falcon 500) no barramento CAN padrao ou CANivore.
   */
  public static MotorController createTalonFX(int canId, String canBus, MotorConfig config) {
    return new TalonFXMotor(canId, canBus, config);
  }

  /**
   * Cria um Talon FX no barramento CAN padrao do RoboRIO.
   */
  public static MotorController createTalonFX(int canId, MotorConfig config) {
    return new TalonFXMotor(canId, "", config);
  }

  /**
   * Cria um Spark MAX (para motores NEO / NEO 550).
   */
  public static MotorController createSparkMax(int canId, MotorConfig config) {
    return new SparkMotor(canId, MotorType.SPARK_MAX, config);
  }

  /**
   * Cria um Spark FLEX (para motores NEO Vortex).
   */
  public static MotorController createSparkFlex(int canId, MotorConfig config) {
    return new SparkMotor(canId, MotorType.SPARK_FLEX, config);
  }

  /**
   * Cria um motor simulado com modelo DC fisico customizado.
   */
  public static MotorController createSimulated(String name, DCMotor motorModel, MotorConfig config) {
    return new SimulatedMotor(name, motorModel, config);
  }

  /**
   * Cria um motor simulado padrao (Kraken X60 1:1).
   */
  public static MotorController createSimulated(String name, MotorConfig config) {
    return new SimulatedMotor(name, config);
  }
}
