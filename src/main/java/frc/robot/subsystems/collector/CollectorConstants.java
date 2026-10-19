package frc.robot.subsystems.collector;

import frc.robot.lib.motors.MotorType;

/**
 * Constantes do sistema de coleta (rolo com 1 motor NEO Vortex + SPARK Flex).
 * TODO: ID CAN e velocidade sao PLACEHOLDERS: confirmar no robo.
 */
public final class CollectorConstants {
  private CollectorConstants() {}

  // --- Hardware ---
  public static final String kCanBus = "";
  public static final MotorType kMotorType = MotorType.SPARK_FLEX; // NEO Vortex
  /** Mesmo ID do rascunho em Constants.IntakeConstants (23). Confirme no robo. */
  public static final int kRollerCanId = 23;

  /** Reducao entre o motor e o rolo (rotacoes do motor / rotacoes do rolo). */
  public static final double kRollerGearRatio = 1.0;

  // --- Sentido de giro ---
  /**
   * Com false o Vortex gira no sentido PADRAO: anti-horario visto de frente para o eixo de saida
   * do motor. Se no robo o rolo coletar para o lado errado, mude para true.
   */
  public static final boolean kRollerInverted = false;

  // --- Saida (fracao da tensao, 0..1) ---
  public static final double kCollectOutput = 0.80;

  // --- Limites de corrente (A) ---
  public static final double kRollerCurrentLimitAmps = 40.0;
}
