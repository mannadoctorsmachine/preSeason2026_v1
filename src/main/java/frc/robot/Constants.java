// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** Modo de execucao do codigo, usado para configurar o AdvantageKit. */
  public enum Mode {
    /** Rodando no robo real. */
    REAL,
    /** Rodando em simulacao no computador. */
    SIM,
    /** Reprocessando um log (replay) no computador. */
    REPLAY
  }

  /** Mude para {@link Mode#REPLAY} para reexecutar um arquivo de log no simulador. */
  private static final Mode kSimMode = Mode.SIM;

  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

  public static class OperatorConstants {
    /** Controle do PILOTO: controla somente o swerve. */
    public static final int kDriverControllerPort = 0;

    /** Controle do OPERADOR: controla os subsistemas (coleta, intake, shooter...). */
    public static final int kOperatorControllerPort = 1;
  }

  public static class DriveConstants {
    public static final double kMaxSpeedMps = 4.5;

    /** Velocidade angular maxima (rad/s). */
    public static final double kMaxAngularSpeedRadPerSec = Math.PI * 2.0;

    // --- Teleoperacao do piloto (FieldDriveCommand / FreedomDriveCommand) ---
    /** Zona morta dos analogicos (0..1). */
    public static final double kStickDeadband = 0.08;

    /**
     * Fracao da velocidade maxima SEM apertar o RT. Apertando o RT (analogico) a velocidade sobe
     * linearmente ate 100%. Com 0.0 o robo so anda enquanto o RT estiver apertado.
     */
    public static final double kBaseSpeedFraction = 0.40;

    /** Fracao da velocidade angular maxima nos giros (nao depende do RT). */
    public static final double kRotationSpeedFraction = 0.60;

    /** Zona morta do gatilho RT (0..1). */
    public static final double kBoostDeadband = 0.05;
  }

  public static class IntakeConstants {
    public static final int kCanId = 23;
    public static final String kCanBus = ""; // Barramento CAN padrao do RoboRIO
    public static final frc.robot.lib.motors.MotorType kMotorType =
        frc.robot.lib.motors.MotorType.SPARK_MAX;
    public static final boolean kInverted = false;
    public static final double kCurrentLimitAmps = 35.0;

    // Velocidades padrao
    public static final double kIntakeSpeed = 0.8;
    public static final double kOuttakeSpeed = -0.6;
  }
}
