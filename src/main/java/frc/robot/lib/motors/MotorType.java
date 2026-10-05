package frc.robot.lib.motors;

/**
 * Tipos de controladores de motores suportados pela biblioteca.
 */
public enum MotorType {
  TALON_FX,     // Kraken X60, Falcon 500 (CTRE Phoenix 6)
  SPARK_MAX,    // NEO, NEO 550 (REV Robotics)
  SPARK_FLEX,   // NEO Vortex (REV Robotics)
  SIMULATED     // Modelo de simulacao puramente em software
}
