package frc.robot.lib.motors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Implementacao de {@link MotorController} para controladores REV Robotics (Spark MAX e Spark FLEX)
 * compativel com a nova API REVLib.
 */
public class SparkMotor implements MotorController {
  private final SparkBase spark;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;
  private final int canId;
  private final MotorType motorType;
  private MotorConfig currentConfig;

  public SparkMotor(int canId, MotorType type, MotorConfig config) {
    this.canId = canId;
    this.motorType = type;

    if (type == MotorType.SPARK_FLEX) {
      this.spark = new SparkFlex(canId, SparkLowLevel.MotorType.kBrushless);
    } else {
      this.spark = new SparkMax(canId, SparkLowLevel.MotorType.kBrushless);
    }

    this.closedLoopController = spark.getClosedLoopController();
    this.encoder = spark.getEncoder();

    applyConfiguration(config);
  }

  @Override
  public void applyConfiguration(MotorConfig config) {
    this.currentConfig = config;
    SparkBaseConfig sparkConfig = (motorType == MotorType.SPARK_FLEX) ? new SparkFlexConfig() : new SparkMaxConfig();

    // Modo Neutro e Inversao
    sparkConfig.idleMode((config.neutralMode == NeutralMode.BRAKE) ? IdleMode.kBrake : IdleMode.kCoast);
    sparkConfig.inverted(config.inverted);

    // Limites de Corrente
    if (config.enableCurrentLimit) {
      sparkConfig.smartCurrentLimit((int) config.statorCurrentLimitAmps);
    }

    // Compensacao de tensao e rampas
    sparkConfig.voltageCompensation(config.voltageCompensation);
    sparkConfig.openLoopRampRate(config.openLoopRampRateSeconds);
    sparkConfig.closedLoopRampRate(config.closedLoopRampRateSeconds);

    // Fatores de conversao para o eixo de saida (1 rotacao do mecanismo = 1 / gearRatio rotacoes do motor)
    double positionConversionFactor = 1.0 / config.gearRatio;
    // Velocidade de RPM para RPS no eixo de saida
    double velocityConversionFactor = (1.0 / config.gearRatio) / 60.0;

    sparkConfig.encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor);

    // Configuracao da Malha Fechada (PID)
    sparkConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(config.pid.kP)
        .i(config.pid.kI)
        .d(config.pid.kD)
        .iZone(config.pid.iZone)
        .outputRange(-config.pid.peakOutput, config.pid.peakOutput);

    // Feedforward estatico/velocidade (kS, kV)
    sparkConfig.closedLoop.velocityFF(config.pid.kV);

    // Motion Profile (MAXMotion)
    if (config.motionProfile.cruiseVelocityRotPerSec > 0) {
      sparkConfig.closedLoop.maxMotion
          .maxVelocity(config.motionProfile.cruiseVelocityRotPerSec)
          .maxAcceleration(config.motionProfile.accelerationRotPerSecSq)
          .allowedClosedLoopError(config.pid.tolerance);
    }

    // Limites de Software
    if (config.enableForwardSoftLimit) {
      sparkConfig.softLimit
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(config.forwardSoftLimitRotations);
    } else {
      sparkConfig.softLimit.forwardSoftLimitEnabled(false);
    }

    if (config.enableReverseSoftLimit) {
      sparkConfig.softLimit
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(config.reverseSoftLimitRotations);
    } else {
      sparkConfig.softLimit.reverseSoftLimitEnabled(false);
    }

    // Continuous Wrap (estercamento)
    if (config.continuousWrap) {
      sparkConfig.closedLoop
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(config.continuousWrapMinRotations, config.continuousWrapMaxRotations);
    } else {
      sparkConfig.closedLoop.positionWrappingEnabled(false);
    }

    spark.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public MotorConfig getConfiguration() {
    return currentConfig;
  }

  @Override
  public void setPercentOutput(double percent) {
    spark.set(percent);
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void setPosition(double targetPositionRotations) {
    setPosition(targetPositionRotations, 0.0);
  }

  @Override
  public void setPosition(double targetPositionRotations, double arbitraryFeedforwardVolts) {
    if (arbitraryFeedforwardVolts != 0.0) {
      closedLoopController.setReference(
          targetPositionRotations,
          ControlType.kPosition,
          com.revrobotics.spark.ClosedLoopSlot.kSlot0,
          arbitraryFeedforwardVolts,
          ArbFFUnits.kVoltage);
    } else {
      closedLoopController.setReference(targetPositionRotations, ControlType.kPosition);
    }
  }

  @Override
  public void setVelocity(double targetVelocityRotPerSec) {
    setVelocity(targetVelocityRotPerSec, 0.0);
  }

  @Override
  public void setVelocity(double targetVelocityRotPerSec, double arbitraryFeedforwardVolts) {
    if (arbitraryFeedforwardVolts != 0.0) {
      closedLoopController.setReference(
          targetVelocityRotPerSec,
          ControlType.kVelocity,
          com.revrobotics.spark.ClosedLoopSlot.kSlot0,
          arbitraryFeedforwardVolts,
          ArbFFUnits.kVoltage);
    } else {
      closedLoopController.setReference(targetVelocityRotPerSec, ControlType.kVelocity);
    }
  }

  @Override
  public void setMotionProfilePosition(double targetPositionRotations) {
    setMotionProfilePosition(targetPositionRotations, 0.0);
  }

  @Override
  public void setMotionProfilePosition(double targetPositionRotations, double arbitraryFeedforwardVolts) {
    if (arbitraryFeedforwardVolts != 0.0) {
      closedLoopController.setReference(
          targetPositionRotations,
          ControlType.kMAXMotionPositionControl,
          com.revrobotics.spark.ClosedLoopSlot.kSlot0,
          arbitraryFeedforwardVolts,
          ArbFFUnits.kVoltage);
    } else {
      closedLoopController.setReference(targetPositionRotations, ControlType.kMAXMotionPositionControl);
    }
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean brake) {
    SparkBaseConfig tempConfig = (motorType == MotorType.SPARK_FLEX) ? new SparkFlexConfig() : new SparkMaxConfig();
    tempConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    spark.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setInverted(boolean inverted) {
    SparkBaseConfig tempConfig = (motorType == MotorType.SPARK_FLEX) ? new SparkFlexConfig() : new SparkMaxConfig();
    tempConfig.inverted(inverted);
    spark.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setEncoderPosition(double positionRotations) {
    encoder.setPosition(positionRotations);
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public double getAppliedVoltage() {
    return spark.getAppliedOutput() * spark.getBusVoltage();
  }

  @Override
  public double getStatorCurrent() {
    return spark.getOutputCurrent();
  }

  @Override
  public double getSupplyCurrent() {
    // Estimativa com base em output percent e corrente de saida
    return Math.abs(spark.getAppliedOutput()) * spark.getOutputCurrent();
  }

  @Override
  public double getTemperature() {
    return spark.getMotorTemperature();
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    inputs.positionRotations = encoder.getPosition();
    inputs.positionRadians = inputs.positionRotations * (2.0 * Math.PI);
    inputs.velocityRotationsPerSec = encoder.getVelocity();
    inputs.velocityRadiansPerSec = inputs.velocityRotationsPerSec * (2.0 * Math.PI);
    inputs.appliedVolts = getAppliedVoltage();
    inputs.statorCurrentAmps = spark.getOutputCurrent();
    inputs.supplyCurrentAmps = inputs.statorCurrentAmps * Math.abs(spark.getAppliedOutput());
    inputs.tempCelsius = spark.getMotorTemperature();
    inputs.isBraking = currentConfig.neutralMode == NeutralMode.BRAKE;
    inputs.faulted = spark.hasActiveFault();
  }

  @Override
  public void follow(MotorController master, boolean opposeMasterDirection) {
    if (master instanceof SparkMotor sparkMaster) {
      SparkBaseConfig tempConfig = (motorType == MotorType.SPARK_FLEX) ? new SparkFlexConfig() : new SparkMaxConfig();
      tempConfig.follow(sparkMaster.spark, opposeMasterDirection);
      spark.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    } else {
      throw new IllegalArgumentException(
          "SparkMotor so pode seguir outro SparkMotor diretamente via hardware.");
    }
  }

  @Override
  public Object getNativeMotor() {
    return spark;
  }

  public int getCanId() {
    return canId;
  }
}
