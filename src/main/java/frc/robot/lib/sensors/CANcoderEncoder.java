package frc.robot.lib.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Implementacao de {@link AbsoluteEncoder} para o CTRE CANcoder (Phoenix 6).
 * Offset, direcao e faixa sao aplicados no proprio hardware via configuracao.
 */
public class CANcoderEncoder implements AbsoluteEncoder {
  private final CANcoder cancoder;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private boolean connected = false;

  public CANcoderEncoder(int canId, String canBus, AbsoluteEncoderConfig config) {
    this.cancoder =
        (canBus == null || canBus.isEmpty())
            ? new CANcoder(canId)
            : new CANcoder(canId, new CANBus(canBus));

    CANcoderConfiguration cfg = new CANcoderConfiguration();
    cfg.MagnetSensor.MagnetOffset = -config.offsetRotations;
    cfg.MagnetSensor.SensorDirection =
        config.inverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    // Faixa [-0.5, 0.5) -> descontinuidade em 0.5 ; faixa [0, 1) -> descontinuidade em 1.0
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = config.minRotations + 1.0;
    cancoder.getConfigurator().apply(cfg);

    positionSignal = cancoder.getAbsolutePosition();
    velocitySignal = cancoder.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, positionSignal, velocitySignal);
    cancoder.optimizeBusUtilization();
  }

  public CANcoderEncoder(int canId, AbsoluteEncoderConfig config) {
    this(canId, "", config);
  }

  private void refresh() {
    connected = BaseStatusSignal.refreshAll(positionSignal, velocitySignal).isOK();
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
  public boolean isConnected() {
    return connected;
  }

  @Override
  public void updateInputs(AbsoluteEncoderInputs inputs) {
    refresh();
    inputs.connected = connected;
    inputs.positionRotations = positionSignal.getValueAsDouble();
    inputs.positionRadians = inputs.positionRotations * 2.0 * Math.PI;
    inputs.velocityRotationsPerSec = velocitySignal.getValueAsDouble();
  }

  @Override
  public Object getNativeSensor() {
    return cancoder;
  }
}
