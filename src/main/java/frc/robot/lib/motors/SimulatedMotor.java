package frc.robot.lib.motors;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Implementacao simulada de {@link MotorController} para execucao em desktop e testes unitarios.
 * Simula a fisica do motor eletrico (tensao, resistencia, inercia, velocidade e posicao).
 */
public class SimulatedMotor implements MotorController {
  private final String name;
  private final DCMotor motorModel;
  private MotorConfig currentConfig;

  private double positionRotations = 0.0;
  private double velocityRotPerSec = 0.0;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;
  private double tempCelsius = 25.0;

  private final PIDController internalPid = new PIDController(0, 0, 0);
  private ControlMode currentMode = ControlMode.PERCENT;
  private double targetReference = 0.0;
  private double arbitraryFeedforward = 0.0;

  private enum ControlMode {
    PERCENT,
    VOLTAGE,
    POSITION,
    VELOCITY,
    MOTION_PROFILE
  }

  public SimulatedMotor(String name, DCMotor motorModel, MotorConfig config) {
    this.name = name;
    this.motorModel = (motorModel != null) ? motorModel : DCMotor.getKrakenX60(1);
    applyConfiguration(config);
  }

  public SimulatedMotor(String name, MotorConfig config) {
    this(name, DCMotor.getKrakenX60(1), config);
  }

  @Override
  public void applyConfiguration(MotorConfig config) {
    this.currentConfig = config;
    internalPid.setPID(config.pid.kP, config.pid.kI, config.pid.kD);
    if (config.continuousWrap) {
      internalPid.enableContinuousInput(
          config.continuousWrapMinRotations, config.continuousWrapMaxRotations);
    } else {
      internalPid.disableContinuousInput();
    }
    if (config.pid.tolerance > 0) {
      internalPid.setTolerance(config.pid.tolerance);
    }
  }

  @Override
  public MotorConfig getConfiguration() {
    return currentConfig;
  }

  @Override
  public void setPercentOutput(double percent) {
    currentMode = ControlMode.PERCENT;
    setVoltage(percent * RobotController.getBatteryVoltage());
  }

  @Override
  public void setVoltage(double volts) {
    currentMode = ControlMode.VOLTAGE;
    this.appliedVolts = Math.max(-12.0, Math.min(12.0, volts));
  }

  @Override
  public void setPosition(double targetPositionRotations) {
    setPosition(targetPositionRotations, 0.0);
  }

  @Override
  public void setPosition(double targetPositionRotations, double arbitraryFeedforwardVolts) {
    currentMode = ControlMode.POSITION;
    this.targetReference = targetPositionRotations;
    this.arbitraryFeedforward = arbitraryFeedforwardVolts;
  }

  @Override
  public void setVelocity(double targetVelocityRotPerSec) {
    setVelocity(targetVelocityRotPerSec, 0.0);
  }

  @Override
  public void setVelocity(double targetVelocityRotPerSec, double arbitraryFeedforwardVolts) {
    currentMode = ControlMode.VELOCITY;
    this.targetReference = targetVelocityRotPerSec;
    this.arbitraryFeedforward = arbitraryFeedforwardVolts;
  }

  @Override
  public void setMotionProfilePosition(double targetPositionRotations) {
    setMotionProfilePosition(targetPositionRotations, 0.0);
  }

  @Override
  public void setMotionProfilePosition(double targetPositionRotations, double arbitraryFeedforwardVolts) {
    currentMode = ControlMode.MOTION_PROFILE;
    this.targetReference = targetPositionRotations;
    this.arbitraryFeedforward = arbitraryFeedforwardVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    currentConfig.neutralMode = brake ? NeutralMode.BRAKE : NeutralMode.COAST;
  }

  @Override
  public void setInverted(boolean inverted) {
    currentConfig.inverted = inverted;
  }

  @Override
  public void setEncoderPosition(double positionRotations) {
    this.positionRotations = positionRotations;
  }

  @Override
  public double getPosition() {
    return positionRotations;
  }

  @Override
  public double getVelocity() {
    return velocityRotPerSec;
  }

  @Override
  public double getAppliedVoltage() {
    return appliedVolts;
  }

  @Override
  public double getStatorCurrent() {
    return currentAmps;
  }

  @Override
  public double getSupplyCurrent() {
    return currentAmps;
  }

  @Override
  public double getTemperature() {
    return tempCelsius;
  }

  /**
   * Executa a integracao fisica em simulacao (periodo nominal dt = 0.02s).
   * @param dtSeconds Passo de tempo da simulacao em segundos.
   */
  public void updateSimulation(double dtSeconds) {
    switch (currentMode) {
      case POSITION, MOTION_PROFILE -> {
        double pidOutput = internalPid.calculate(positionRotations, targetReference);
        appliedVolts = pidOutput + arbitraryFeedforward;
      }
      case VELOCITY -> {
        double pidOutput = internalPid.calculate(velocityRotPerSec, targetReference);
        appliedVolts = pidOutput + arbitraryFeedforward + (currentConfig.pid.kV * targetReference);
      }
      default -> {}
    }

    // Limita saturacao de tensao
    appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));

    // Modelo simples de resposta de 1a ordem do motor
    // Max motor free speed: motorModel.freeSpeedRadPerSec / (2*PI * gearRatio)
    double maxSpeedRPS = (motorModel.freeSpeedRadPerSec / (2.0 * Math.PI)) / currentConfig.gearRatio;
    double targetSpeedRPS = (appliedVolts / 12.0) * maxSpeedRPS;

    if (currentConfig.inverted) {
      targetSpeedRPS = -targetSpeedRPS;
    }

    // Aceleracao suave em direção à velocidade alvo
    double accelerationRate = 40.0; // rps^2
    double speedDiff = targetSpeedRPS - velocityRotPerSec;
    velocityRotPerSec += Math.signum(speedDiff) * Math.min(Math.abs(speedDiff), accelerationRate * dtSeconds);

    if (currentConfig.neutralMode == NeutralMode.BRAKE && Math.abs(appliedVolts) < 0.1) {
      velocityRotPerSec *= 0.85; // freia mais rapido
    }

    // Integracao da posicao
    positionRotations += velocityRotPerSec * dtSeconds;

    // Estimativa de corrente e temperatura
    currentAmps = Math.abs(appliedVolts) * (motorModel.stallCurrentAmps / 12.0) * 0.15;
    tempCelsius = 25.0 + (currentAmps * 0.2);
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    updateSimulation(0.02);

    inputs.positionRotations = positionRotations;
    inputs.positionRadians = positionRotations * (2.0 * Math.PI);
    inputs.velocityRotationsPerSec = velocityRotPerSec;
    inputs.velocityRadiansPerSec = velocityRotPerSec * (2.0 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = currentAmps;
    inputs.supplyCurrentAmps = currentAmps;
    inputs.tempCelsius = tempCelsius;
    inputs.isBraking = currentConfig.neutralMode == NeutralMode.BRAKE;
    inputs.faulted = false;
  }

  @Override
  public void follow(MotorController master, boolean opposeMasterDirection) {
    // Modo Seguidor simulado
  }

  @Override
  public Object getNativeMotor() {
    return null;
  }

  public String getName() {
    return name;
  }
}
