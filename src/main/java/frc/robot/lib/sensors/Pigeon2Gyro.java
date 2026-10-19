package frc.robot.lib.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Implementacao de {@link Gyro} para o CTRE Pigeon 2.0 (Phoenix 6).
 * O Pigeon 2 ja entrega yaw continuo, CCW positivo.
 */
public class Pigeon2Gyro implements Gyro {
  private final Pigeon2 pigeon;

  private final StatusSignal<Angle> yawSignal;
  private final StatusSignal<Angle> pitchSignal;
  private final StatusSignal<Angle> rollSignal;
  private final StatusSignal<AngularVelocity> yawVelocitySignal;

  private boolean connected = false;

  public Pigeon2Gyro(int canId, String canBus) {
    this.pigeon =
        (canBus == null || canBus.isEmpty())
            ? new Pigeon2(canId)
            : new Pigeon2(canId, new CANBus(canBus));

    pigeon.getConfigurator().apply(new Pigeon2Configuration());

    yawSignal = pigeon.getYaw();
    pitchSignal = pigeon.getPitch();
    rollSignal = pigeon.getRoll();
    yawVelocitySignal = pigeon.getAngularVelocityZWorld();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, yawSignal, yawVelocitySignal);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, pitchSignal, rollSignal);
    pigeon.optimizeBusUtilization();
  }

  public Pigeon2Gyro(int canId) {
    this(canId, "");
  }

  private void refresh() {
    connected =
        BaseStatusSignal.refreshAll(yawSignal, pitchSignal, rollSignal, yawVelocitySignal)
            .isOK();
  }

  @Override
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(pitchSignal.getValueAsDouble());
  }

  @Override
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(rollSignal.getValueAsDouble());
  }

  @Override
  public double getYawVelocityRadPerSec() {
    return Math.toRadians(yawVelocitySignal.getValueAsDouble());
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    pigeon.setYaw(yaw.getDegrees());
  }

  @Override
  public boolean isConnected() {
    return connected;
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    refresh();
    inputs.connected = connected;
    // Unidades dos sinais Phoenix 6 ja configuradas em graus / graus por segundo
    inputs.yawRadians = Math.toRadians(yawSignal.getValueAsDouble());
    inputs.pitchRadians = Math.toRadians(pitchSignal.getValueAsDouble());
    inputs.rollRadians = Math.toRadians(rollSignal.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Math.toRadians(yawVelocitySignal.getValueAsDouble());
  }

  @Override
  public Object getNativeSensor() {
    return pigeon;
  }
}
