package frc.robot.subsystems.intake;

import frc.robot.lib.motors.MotorType;
import frc.robot.lib.motors.SafePositionMotor;

/**
 * Constantes do pivo do intake (1 motor NEO Vortex + SPARK Flex que apenas desce e sobe).
 * TODO: ID CAN, relacao, angulos e ganhos sao PLACEHOLDERS: medir/ajustar no robo.
 */
public final class IntakeConstants {
  private IntakeConstants() {}

  // --- Hardware ---
  public static final String kCanBus = "";
  public static final MotorType kMotorType = MotorType.SPARK_FLEX; // NEO Vortex
  public static final int kPivotCanId = 21;

  /** Reducao entre o motor e o intake (rotacoes do motor / rotacoes do intake). */
  public static final double kPivotGearRatio = 45.0;

  /**
   * Sentido positivo do pivo = descer. Se ao mandar DESCER o intake subir (ou o guarda de seguranca
   * acusar WRONG_DIRECTION), mude para true.
   */
  public static final boolean kPivotInverted = false;

  public static final double kPivotCurrentLimitAmps = 30.0;

  // --- Posicoes (graus do intake; 0 = recolhido) ---
  /**
   * O robo deve ser LIGADO com o intake recolhido (UP): o encoder do motor e relativo e assume esta
   * posicao ao ligar.
   */
  public static final double kUpDegrees = 0.0;

  /** Posicao abaixada. Ajuste um pouco ANTES do batente para nao forcar o motor contra ele. */
  public static final double kDownDegrees = 105.0;

  // --- Controle (volts) ---
  public static final double kVortexFreeSpeedRpm = 6784.0;

  /** V por (rotacao/s do intake): 12 V / velocidade livre do intake. */
  public static final double kPivotKv =
      SafePositionMotor.kVFromFreeSpeed(kVortexFreeSpeedRpm / 60.0 / kPivotGearRatio);

  /** Tensao estatica (V). Comece em 0 e aumente so se o intake demorar para comecar a mexer. */
  public static final double kPivotKs = 0.0;

  /** V por rotacao de erro e V por (rotacao/s). Ajustar no robo. */
  public static final double kPivotKp = 20.0;

  public static final double kPivotKd = 0.3;

  /** Compensacao de gravidade (V). 0 = desligada. Ajustar se o intake cair/subir pesado. */
  public static final double kPivotKg = 0.0;

  /** Tensao maxima do pivo (V): limita forca e velocidade (nao bate forte nos batentes). */
  public static final double kPivotMaxVolts = 4.0;

  /** Perfil de movimento (rotacoes do intake por segundo / por segundo ao quadrado). */
  public static final double kPivotMaxVelocityRotPerSec = 0.5;

  public static final double kPivotMaxAccelerationRotPerSecSq = 2.0;

  public static final double kPivotToleranceDegrees = 2.0;

  // --- Protecao contra clique duplo ---
  /**
   * Depois de um pedido aceito, qualquer outro pedido (inclusive o botao oposto) e ignorado por este
   * tempo (s). Pedir a MESMA posicao em que o intake ja esta (ou para onde ja vai) tambem e ignorado.
   */
  public static final double kRequestCooldownSeconds = 0.40;
}
