package frc.robot.subsystems.intake;

import frc.robot.lib.motors.MotorType;
import frc.robot.lib.motors.SafePositionMotor;

/**
 * Constantes do pivo do intake (1 motor NEO Vortex + SPARK Flex que apenas fecha e abre).
 *
 * <p>As posicoes sao o valor do ENCODER PRIMARIO do SPARK (o mesmo numero que o REV Hardware Client
 * mostra em "Position"), em rotacoes do motor. Nao ha conversao por graus nem por relacao.
 * TODO: ID CAN, posicao aberta e ganhos sao PLACEHOLDERS: ajustar no robo.
 */
public final class IntakeConstants {
  private IntakeConstants() {}

  // --- Hardware ---
  public static final String kCanBus = "";
  public static final MotorType kMotorType = MotorType.SPARK_FLEX; // NEO Vortex
  public static final int kPivotCanId = 21;

  /**
   * Sentido positivo do pivo = ABRIR (posicao aumenta). Se ao mandar ABRIR o intake fechar (ou o
   * guarda de seguranca acusar WRONG_DIRECTION), mude para true.
   */
  public static final boolean kPivotInverted = false;

  public static final double kPivotCurrentLimitAmps = 30.0;

  // --- Posicoes (valor do encoder primario, em rotacoes do motor) ---
  /**
   * FECHADO (recolhido). O robo deve ser LIGADO com o intake fechado: o encoder do motor e relativo
   * e assume este valor quando o codigo inicia.
   */
  public static final double kClosedPosition = 0.0;

  /** ABERTO (abaixado). Ajuste para ficar um pouco ANTES do batente. */
  public static final double kOpenPosition = 3.6;

  // --- Controle (volts; posicao e velocidade em rotacoes do motor) ---
  public static final double kVortexFreeSpeedRpm = 6784.0;

  /** V por (rotacao/s do motor): 12 V / velocidade livre do motor. */
  public static final double kPivotKv =
      SafePositionMotor.kVFromFreeSpeed(kVortexFreeSpeedRpm / 60.0);

  /** Tensao estatica (V). Comece em 0 e aumente so se o intake demorar para comecar a mexer. */
  public static final double kPivotKs = 0.0;

  /** V por rotacao de erro e V por (rotacao/s). Ajustar no robo. */
  public static final double kPivotKp = 8.0;

  public static final double kPivotKd = 0.05;

  /** Compensacao de gravidade (V). 0 = desligada. Ajustar se o intake cair/subir pesado. */
  public static final double kPivotKg = 0.0;

  /** Tensao maxima do pivo (V): limita forca e velocidade (nao bate forte nos batentes). */
  public static final double kPivotMaxVolts = 4.0;

  /** Perfil de movimento (rotacoes do motor por segundo / por segundo ao quadrado). */
  public static final double kPivotMaxVelocityRotPerSec = 8.0;

  public static final double kPivotMaxAccelerationRotPerSecSq = 30.0;

  /** Distancia ao alvo (rotacoes do motor) para considerar que chegou. */
  public static final double kPivotTolerance = 0.05;

  // --- Protecao contra clique duplo ---
  /**
   * Depois de um pedido aceito, qualquer outro pedido (inclusive o botao oposto) e ignorado por este
   * tempo (s). Pedir a MESMA posicao em que o intake ja esta (ou para onde ja vai) tambem e ignorado.
   */
  public static final double kRequestCooldownSeconds = 0.40;
}
