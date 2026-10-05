package frc.robot.lib.motors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;



/**
 * Implementacao de {@link MotorController} para controladores CTRE Talon FX (Kraken X60, Falcon 500)
 * utilizando o Phoenix 6.
 */
public class TalonFXMotor implements MotorController {
  private final TalonFX talonFX;
  private final String canBus;
  private final int canId;
  private MotorConfig currentConfig;

  // Requisicoes de controle reutilizaveis para evitar alocacao de memoria no loop periodico
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  // Sinais de telemetria de alta frequencia
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> appliedVoltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Temperature> deviceTempSignal;

  public TalonFXMotor(int canId, String canBus, MotorConfig config) {
    this.canId = canId;
    this.canBus = canBus;
    this.talonFX = (canBus == null || canBus.isEmpty())
        ? new TalonFX(canId)
        : new TalonFX(canId, new CANBus(canBus));

    this.positionSignal = talonFX.getPosition();
    this.velocitySignal = talonFX.getVelocity();
    this.appliedVoltageSignal = talonFX.getMotorVoltage();
    this.statorCurrentSignal = talonFX.getStatorCurrent();
    this.supplyCurrentSignal = talonFX.getSupplyCurrent();
    this.deviceTempSignal = talonFX.getDeviceTemp();

    // Otimiza taxas de atualizacao dos sinais
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionSignal,
        velocitySignal,
        appliedVoltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        deviceTempSignal);

    talonFX.optimizeBusUtilization();

    applyConfiguration(config);
  }

  public TalonFXMotor(int canId, MotorConfig config) {
    this(canId, "", config);
  }

  @Override
  public void applyConfiguration(MotorConfig config) {
    this.currentConfig = config;
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    // Modo neutro e inversao
    talonConfig.MotorOutput.NeutralMode =
        (config.neutralMode == NeutralMode.BRAKE) ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talonConfig.MotorOutput.Inverted =
        config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Rampas
    talonConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
    talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.openLoopRampRateSeconds;
    talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.openLoopRampRateSeconds;
    talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = config.closedLoopRampRateSeconds;
    talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.closedLoopRampRateSeconds;

    // Limites de Corrente
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = config.enableCurrentLimit;
    talonConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimitAmps;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = config.enableCurrentLimit;
    talonConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimitAmps;

    // Relacao de transmissao (Sensor to Mechanism Ratio)
    talonConfig.Feedback.SensorToMechanismRatio = config.gearRatio;

    // Ganhos PID + Feedforward (Slot 0)
    talonConfig.Slot0.kP = config.pid.kP;
    talonConfig.Slot0.kI = config.pid.kI;
    talonConfig.Slot0.kD = config.pid.kD;
    talonConfig.Slot0.kS = config.pid.kS;
    talonConfig.Slot0.kV = config.pid.kV;
    talonConfig.Slot0.kA = config.pid.kA;
    talonConfig.Slot0.kG = config.pid.kG;

    // Perfil Motion Magic
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = config.motionProfile.cruiseVelocityRotPerSec;
    talonConfig.MotionMagic.MotionMagicAcceleration = config.motionProfile.accelerationRotPerSecSq;
    talonConfig.MotionMagic.MotionMagicJerk = config.motionProfile.jerkRotPerSecCubed;

    // Soft Limits
    talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = config.enableForwardSoftLimit;
    talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.forwardSoftLimitRotations;
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = config.enableReverseSoftLimit;
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.reverseSoftLimitRotations;

    // Continuous wrap para giro contínuo (ex: estercamento swerve)
    talonConfig.ClosedLoopGeneral.ContinuousWrap = config.continuousWrap;

    talonFX.getConfigurator().apply(talonConfig);
  }

  @Override
  public MotorConfig getConfiguration() {
    return currentConfig;
  }

  @Override
  public void setPercentOutput(double percent) {
    talonFX.setControl(dutyCycleRequest.withOutput(percent));
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double targetPositionRotations) {
    setPosition(targetPositionRotations, 0.0);
  }

  @Override
  public void setPosition(double targetPositionRotations, double arbitraryFeedforwardVolts) {
    talonFX.setControl(
        positionRequest
            .withPosition(targetPositionRotations)
            .withFeedForward(arbitraryFeedforwardVolts));
  }

  @Override
  public void setVelocity(double targetVelocityRotPerSec) {
    setVelocity(targetVelocityRotPerSec, 0.0);
  }

  @Override
  public void setVelocity(double targetVelocityRotPerSec, double arbitraryFeedforwardVolts) {
    talonFX.setControl(
        velocityRequest
            .withVelocity(targetVelocityRotPerSec)
            .withFeedForward(arbitraryFeedforwardVolts));
  }

  @Override
  public void setMotionProfilePosition(double targetPositionRotations) {
    setMotionProfilePosition(targetPositionRotations, 0.0);
  }

  @Override
  public void setMotionProfilePosition(double targetPositionRotations, double arbitraryFeedforwardVolts) {
    talonFX.setControl(
        motionMagicRequest
            .withPosition(targetPositionRotations)
            .withFeedForward(arbitraryFeedforwardVolts));
  }

  @Override
  public void stop() {
    talonFX.setControl(neutralRequest);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    talonFX.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setInverted(boolean inverted) {
    // Phoenix 6: alteracao em runtime requer aplicar a configuracao via Configurator
    var config = new TalonFXConfiguration();
    talonFX.getConfigurator().refresh(config);
    config.MotorOutput.Inverted = inverted
        ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
        : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    talonFX.getConfigurator().apply(config.MotorOutput);
    currentConfig.inverted = inverted;
  }

  @Override
  public void setEncoderPosition(double positionRotations) {
    talonFX.setPosition(positionRotations);
  }

  @Override
  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  @Override
  public double getAppliedVoltage() {
    return appliedVoltageSignal.getValueAsDouble();
  }

  @Override
  public double getStatorCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  @Override
  public double getSupplyCurrent() {
    return supplyCurrentSignal.getValueAsDouble();
  }

  @Override
  public double getTemperature() {
    return deviceTempSignal.getValueAsDouble();
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        appliedVoltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        deviceTempSignal);

    inputs.positionRotations = positionSignal.getValueAsDouble();
    inputs.positionRadians = inputs.positionRotations * (2.0 * Math.PI);
    inputs.velocityRotationsPerSec = velocitySignal.getValueAsDouble();
    inputs.velocityRadiansPerSec = inputs.velocityRotationsPerSec * (2.0 * Math.PI);
    inputs.appliedVolts = appliedVoltageSignal.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.tempCelsius = deviceTempSignal.getValueAsDouble();
    inputs.isBraking = currentConfig.neutralMode == NeutralMode.BRAKE;
    inputs.faulted = talonFX.getFaultField().getValueAsDouble() != 0;
  }

  @Override
  public void follow(MotorController master, boolean opposeMasterDirection) {
    if (master instanceof TalonFXMotor talonMaster) {
      com.ctre.phoenix6.signals.MotorAlignmentValue alignment = opposeMasterDirection
          ? com.ctre.phoenix6.signals.MotorAlignmentValue.Opposed
          : com.ctre.phoenix6.signals.MotorAlignmentValue.Aligned;
      talonFX.setControl(new Follower(talonMaster.talonFX.getDeviceID(), alignment));
    } else {
      throw new IllegalArgumentException(
          "TalonFX so pode seguir outro TalonFX diretamente via hardware.");
    }
  }

  @Override
  public Object getNativeMotor() {
    return talonFX;
  }

  public int getCanId() {
    return canId;
  }

  public String getCanBus() {
    return canBus;
  }
}
