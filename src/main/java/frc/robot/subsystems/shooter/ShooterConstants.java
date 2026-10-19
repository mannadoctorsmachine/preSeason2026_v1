package frc.robot.subsystems.shooter;

import frc.robot.lib.motors.MotorType;
import frc.robot.lib.motors.SafeVelocityMotor;

/**
 * Constantes do conjunto Index -> Intermediario -> Shooter (3 motores).
 * TODO: IDs CAN, relacoes, sentidos e ganhos sao PLACEHOLDERS: medir/ajustar no robo.
 *
 * <p>Todas as RPM sao no eixo de SAIDA de cada estagio (rpm do motor / relacao). Com relacao 1.0 sao
 * as RPM do proprio motor.
 */
public final class ShooterConstants {
  private ShooterConstants() {}

  // --- Hardware ---
  public static final String kCanBus = "";
  public static final MotorType kMotorType = MotorType.SPARK_FLEX; // NEO Vortex
  public static final int kIndexCanId = 22;
  public static final int kIntermediateCanId = 24;
  public static final int kShooterCanId = 25;

  /** Reducao (rotacoes do motor / rotacoes do estagio). */
  public static final double kIndexGearRatio = 1.0;

  public static final double kIntermediateGearRatio = 1.0;
  public static final double kShooterGearRatio = 1.0;

  /**
   * Sentido de giro de cada estagio: +1 ou -1 (sinal da tensao). A logica de RPM usa o MODULO da
   * velocidade lida, entao funciona mesmo que o encoder leia negativo.
   */
  public static final int kIndexDirection = 1;

  public static final int kIntermediateDirection = 1;
  public static final int kShooterDirection = 1;

  // --- Alvos (RPM). Cada estagio nunca passa do seu valor ---
  public static final double kIndexTargetRpm = 5000.0;
  public static final double kIntermediateTargetRpm = 5000.0;
  public static final double kShooterTargetRpm = 6000.0;

  // --- Sequencia ---
  /**
   * O "RPM x" que libera o proximo estagio: fracao do alvo que o estagio anterior precisa atingir.
   * 0.97 => o Intermediario so liga com o Index em 97% de 5000 = 4850 RPM; o Shooter so liga com o
   * Intermediario em 4850 RPM. Mude para 1.0 se quiser exigir o alvo inteiro.
   */
  public static final double kReachedFraction = 0.97;

  /** Tempo maximo (s) para o Index e o Intermediario atingirem o RPM; senao a sequencia aborta. */
  public static final double kStageTimeoutSeconds = 4.0;

  /** Aceleracao do alvo de cada estagio (RPM por segundo): evita tranco de corrente. */
  public static final double kRampRpmPerSec = 8000.0;

  // --- Controle (volts) ---
  public static final double kVortexFreeSpeedRpm = 6784.0;

  /** Tensao estatica (V) para vencer o atrito. Comece em 0; aumente se demorar a comecar a girar. */
  public static final double kS = 0.0;

  /**
   * Ganho proporcional (V por RPM de erro). Se o RPM ficar abaixo do alvo com carga, aumente (ex:
   * 0.002); se oscilar, diminua.
   */
  public static final double kPVoltsPerRpm = 0.001;

  /**
   * Ganho integral (V por RPM.s): corrige o erro que sobra quando o mecanismo real gira menos do que
   * o feedforward teorico preve (atrito, carga). So atua perto do alvo.
   */
  public static final double kIVoltsPerRpmSecond = 0.004;

  /** Teto de tensao (V). */
  public static final double kMaxVolts = 12.0;

  /** Feedforward teorico (V por RPM) de um Vortex apos a reducao informada. */
  public static double kvForGearRatio(double gearRatio) {
    return SafeVelocityMotor.kVFromFreeSpeed(kVortexFreeSpeedRpm / gearRatio);
  }

  // --- Protecoes ---
  /** Acima do alvo + 2% o motor e solto ate voltar; acima de alvo + 8% ha falha e tudo para. */
  public static final double kSoftOverspeedFraction = 0.02;

  public static final double kHardOverspeedFraction = 0.08;

  /** Travado/sem encoder: tensao >= kStallMinVolts com RPM < kStallMaxRpm por kStallSeconds. */
  public static final double kStallMinVolts = 3.0;

  public static final double kStallMaxRpm = 100.0;
  public static final double kStallSeconds = 0.75;

  // --- Limites de corrente (A) ---
  public static final double kIndexCurrentLimitAmps = 40.0;
  public static final double kIntermediateCurrentLimitAmps = 40.0;
  public static final double kShooterCurrentLimitAmps = 60.0;
}
