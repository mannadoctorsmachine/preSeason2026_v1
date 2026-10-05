package frc.robot.lib.motors;

/**
 * Classe abrangente de configuracao para qualquer motor do robo.
 * Permite definir limites de corrente, modos neutros, PID, rampas e reducao de engrenagens.
 */
public class MotorConfig {
  public NeutralMode neutralMode = NeutralMode.BRAKE;
  public boolean inverted = false;

  // Limites de Corrente
  public boolean enableCurrentLimit = true;
  public double statorCurrentLimitAmps = 40.0;
  public double supplyCurrentLimitAmps = 40.0;

  // Compensacao de Tensao e Rampas
  public double voltageCompensation = 12.0;
  public double openLoopRampRateSeconds = 0.0;
  public double closedLoopRampRateSeconds = 0.0;

  // Relacao de Transmissao: (Rotacoes do Motor / Rotacoes do Mecanismo)
  // Ex: Swerve Drive L3 = 6.12 -> 6.12 rotacoes do motor geram 1 rotacao da roda.
  public double gearRatio = 1.0;

  // Ganhos de Controle e Perfil
  public PIDConfig pid = new PIDConfig();
  public MotionProfileConfig motionProfile = new MotionProfileConfig();

  // Limites de Software (Soft Limits em rotacoes do mecanismo)
  public boolean enableForwardSoftLimit = false;
  public double forwardSoftLimitRotations = 0.0;
  public boolean enableReverseSoftLimit = false;
  public double reverseSoftLimitRotations = 0.0;

  // Envolvimento continuo (Continuous Wrap - util para estercamento de Swerve)
  public boolean continuousWrap = false;
  public double continuousWrapMinRotations = -0.5;
  public double continuousWrapMaxRotations = 0.5;

  public MotorConfig() {}

  public MotorConfig withNeutralMode(NeutralMode neutralMode) {
    this.neutralMode = neutralMode;
    return this;
  }

  public MotorConfig withInverted(boolean inverted) {
    this.inverted = inverted;
    return this;
  }

  public MotorConfig withStatorCurrentLimit(double amps) {
    this.enableCurrentLimit = true;
    this.statorCurrentLimitAmps = amps;
    return this;
  }

  public MotorConfig withSupplyCurrentLimit(double amps) {
    this.enableCurrentLimit = true;
    this.supplyCurrentLimitAmps = amps;
    return this;
  }

  public MotorConfig withVoltageCompensation(double volts) {
    this.voltageCompensation = volts;
    return this;
  }

  public MotorConfig withRampRates(double openLoopSec, double closedLoopSec) {
    this.openLoopRampRateSeconds = openLoopSec;
    this.closedLoopRampRateSeconds = closedLoopSec;
    return this;
  }

  public MotorConfig withGearRatio(double gearRatio) {
    this.gearRatio = gearRatio;
    return this;
  }

  public MotorConfig withPID(PIDConfig pid) {
    this.pid = pid;
    return this;
  }

  public MotorConfig withPID(double kP, double kI, double kD) {
    this.pid = new PIDConfig(kP, kI, kD);
    return this;
  }

  public MotorConfig withPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    this.pid = new PIDConfig(kP, kI, kD, kS, kV, kA);
    return this;
  }

  public MotorConfig withMotionProfile(MotionProfileConfig motionProfile) {
    this.motionProfile = motionProfile;
    return this;
  }

  public MotorConfig withSoftLimits(double reverseRotations, double forwardRotations) {
    this.enableForwardSoftLimit = true;
    this.forwardSoftLimitRotations = forwardRotations;
    this.enableReverseSoftLimit = true;
    this.reverseSoftLimitRotations = reverseRotations;
    return this;
  }

  public MotorConfig withContinuousWrap(double minRotations, double maxRotations) {
    this.continuousWrap = true;
    this.continuousWrapMinRotations = minRotations;
    this.continuousWrapMaxRotations = maxRotations;
    return this;
  }
}
