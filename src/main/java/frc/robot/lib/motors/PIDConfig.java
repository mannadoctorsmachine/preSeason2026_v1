package frc.robot.lib.motors;

/**
 * Configuracoes de ganhos de malha fechada (PID + Feedforward).
 */
public class PIDConfig {
  public double kP = 0.0;
  public double kI = 0.0;
  public double kD = 0.0;

  // Ganhos de Feedforward padrao WPILib / CTRE / REV
  public double kS = 0.0; // Volts estaticos para vencer atrito
  public double kV = 0.0; // Volts por velocidade (V / (rot/s))
  public double kA = 0.0; // Volts por aceleracao (V / (rot/s^2))
  public double kG = 0.0; // Volts para gravidade (ex: bracos/elevadores)

  public double iZone = 0.0;
  public double peakOutput = 1.0; // Saturacao maxima de saida (-1.0 a 1.0 ou proporcao)
  public double tolerance = 0.0;

  public PIDConfig() {}

  public PIDConfig(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  public PIDConfig(double kP, double kI, double kD, double kS, double kV, double kA) {
    this(kP, kI, kD);
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }

  public PIDConfig withP(double kP) {
    this.kP = kP;
    return this;
  }

  public PIDConfig withI(double kI) {
    this.kI = kI;
    return this;
  }

  public PIDConfig withD(double kD) {
    this.kD = kD;
    return this;
  }

  public PIDConfig withFeedforward(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    return this;
  }

  public PIDConfig withGravity(double kG) {
    this.kG = kG;
    return this;
  }

  public PIDConfig withIZone(double iZone) {
    this.iZone = iZone;
    return this;
  }

  public PIDConfig withTolerance(double tolerance) {
    this.tolerance = tolerance;
    return this;
  }
}
