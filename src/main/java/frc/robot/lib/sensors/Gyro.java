package frc.robot.lib.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface unificada para giroscopios/IMUs (Pigeon 2, NavX2 e Simulado).
 * Convencao: angulos positivos = anti-horario (CCW), yaw continuo (nao faz wrap em +-180).
 */
public interface Gyro {

  /** @return Yaw atual como Rotation2d (continuo; Rotation2d normaliza internamente se necessario). */
  Rotation2d getYaw();

  /** @return Pitch atual. */
  Rotation2d getPitch();

  /** @return Roll atual. */
  Rotation2d getRoll();

  /** @return Velocidade angular de guinada em rad/s. */
  double getYawVelocityRadPerSec();

  /**
   * Redefine o yaw atual para o valor informado (ex: zerar a frente do robo ou aplicar pose inicial).
   * @param yaw Novo valor de yaw.
   */
  void setYaw(Rotation2d yaw);

  /** Zera o yaw (frente atual do robo passa a ser 0). */
  default void zeroYaw() {
    setYaw(Rotation2d.kZero);
  }

  /** @return true se o sensor esta conectado e enviando dados. */
  boolean isConnected();

  /**
   * Atualiza a estrutura de telemetria para logging e determinismo de simulacao/replay.
   * @param inputs Objeto de telemetria a ser preenchido.
   */
  void updateInputs(GyroInputs inputs);

  /** @return Objeto nativo do fabricante (Pigeon2, AHRS) ou null. */
  Object getNativeSensor();
}
