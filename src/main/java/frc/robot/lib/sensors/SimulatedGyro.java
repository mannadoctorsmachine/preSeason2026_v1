package frc.robot.lib.sensors;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Implementacao simulada de {@link Gyro}.
 * A fisica externa (ex: simulacao do swerve) alimenta a velocidade angular via
 * {@link #setYawVelocityRadPerSec(double)} e o yaw e integrado a cada updateInputs (dt = 0.02s).
 */
public class SimulatedGyro implements Gyro {
  private double yawRadians = 0.0;
  private double pitchRadians = 0.0;
  private double rollRadians = 0.0;
  private double yawVelocityRadPerSec = 0.0;

  /** Define a velocidade angular simulada (rad/s) que sera integrada no yaw. */
  public void setYawVelocityRadPerSec(double radPerSec) {
    this.yawVelocityRadPerSec = radPerSec;
  }

  /** Define pitch e roll simulados (ex: rampas). */
  public void setPitchRoll(Rotation2d pitch, Rotation2d roll) {
    this.pitchRadians = pitch.getRadians();
    this.rollRadians = roll.getRadians();
  }

  @Override
  public Rotation2d getYaw() {
    return new Rotation2d(yawRadians);
  }

  @Override
  public Rotation2d getPitch() {
    return new Rotation2d(pitchRadians);
  }

  @Override
  public Rotation2d getRoll() {
    return new Rotation2d(rollRadians);
  }

  @Override
  public double getYawVelocityRadPerSec() {
    return yawVelocityRadPerSec;
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    yawRadians = yaw.getRadians();
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    yawRadians += yawVelocityRadPerSec * 0.02;

    inputs.connected = true;
    inputs.yawRadians = yawRadians;
    inputs.pitchRadians = pitchRadians;
    inputs.rollRadians = rollRadians;
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
  }

  @Override
  public Object getNativeSensor() {
    return null;
  }
}
