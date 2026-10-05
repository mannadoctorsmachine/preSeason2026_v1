package frc.robot.lib.motors;

/**
 * Configuracoes de perfil de movimento trapezoidal/curva-S (Motion Magic / MAXMotion / Smart Motion).
 * Todas as unidades referem-se ao eixo de saida do mecanismo (rotacoes de saida).
 */
public class MotionProfileConfig {
  public double cruiseVelocityRotPerSec = 0.0;
  public double accelerationRotPerSecSq = 0.0;
  public double jerkRotPerSecCubed = 0.0;

  public MotionProfileConfig() {}

  public MotionProfileConfig(double cruiseVelocityRotPerSec, double accelerationRotPerSecSq) {
    this.cruiseVelocityRotPerSec = cruiseVelocityRotPerSec;
    this.accelerationRotPerSecSq = accelerationRotPerSecSq;
  }

  public MotionProfileConfig(
      double cruiseVelocityRotPerSec,
      double accelerationRotPerSecSq,
      double jerkRotPerSecCubed) {
    this.cruiseVelocityRotPerSec = cruiseVelocityRotPerSec;
    this.accelerationRotPerSecSq = accelerationRotPerSecSq;
    this.jerkRotPerSecCubed = jerkRotPerSecCubed;
  }

  public MotionProfileConfig withCruiseVelocity(double cruiseVelocityRotPerSec) {
    this.cruiseVelocityRotPerSec = cruiseVelocityRotPerSec;
    return this;
  }

  public MotionProfileConfig withAcceleration(double accelerationRotPerSecSq) {
    this.accelerationRotPerSecSq = accelerationRotPerSecSq;
    return this;
  }

  public MotionProfileConfig withJerk(double jerkRotPerSecCubed) {
    this.jerkRotPerSecCubed = jerkRotPerSecCubed;
    return this;
  }
}
