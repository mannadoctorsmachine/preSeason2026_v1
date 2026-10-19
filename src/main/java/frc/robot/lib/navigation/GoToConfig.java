package frc.robot.lib.navigation;

/** Parametros de ajuste do {@link GoToPoseCommand}. Todos os valores em unidades SI (m, rad, s). */
public class GoToConfig {
  /** Velocidade linear maxima durante o deslocamento (m/s). */
  public double maxVelocityMps = 3.0;

  /** Aceleracao/desaceleracao linear maxima do perfil (m/s^2). */
  public double maxAccelerationMps2 = 3.0;

  /** Ganho proporcional da distancia restante (malha do ProfiledPID). */
  public double distanceKp = 3.0;

  /** Velocidade angular maxima (rad/s) e aceleracao angular maxima (rad/s^2) para girar ao heading final. */
  public double maxAngularVelocityRadPerSec = Math.PI * 2.0;
  public double maxAngularAccelerationRadPerSec2 = Math.PI * 4.0;

  /** Ganho proporcional do heading. */
  public double headingKp = 5.0;

  /** Distancia (m) para considerar um ponto intermediario da rota como atingido. */
  public double waypointToleranceMeters = 0.15;

  /** Tolerancias de chegada ao destino. */
  public double positionToleranceMeters = 0.03;
  public double headingToleranceRad = Math.toRadians(2.0);
  public double velocityToleranceMps = 0.05;

  /**
   * Se true, destino, heading e obstaculos sao informados no referencial AZUL e espelhados
   * automaticamente quando estamos na alianca vermelha.
   */
  public boolean flipForAlliance = true;

  public GoToConfig() {}

  public GoToConfig withAllianceFlip(boolean flip) {
    this.flipForAlliance = flip;
    return this;
  }

  public GoToConfig withLinearLimits(double maxVelocityMps, double maxAccelerationMps2) {
    this.maxVelocityMps = maxVelocityMps;
    this.maxAccelerationMps2 = maxAccelerationMps2;
    return this;
  }

  public GoToConfig withAngularLimits(double maxVelRadPerSec, double maxAccelRadPerSec2) {
    this.maxAngularVelocityRadPerSec = maxVelRadPerSec;
    this.maxAngularAccelerationRadPerSec2 = maxAccelRadPerSec2;
    return this;
  }

  public GoToConfig withGains(double distanceKp, double headingKp) {
    this.distanceKp = distanceKp;
    this.headingKp = headingKp;
    return this;
  }

  public GoToConfig withTolerances(double positionMeters, double headingRad) {
    this.positionToleranceMeters = positionMeters;
    this.headingToleranceRad = headingRad;
    return this;
  }
}
