package frc.robot.lib.sensors;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Implementacao de {@link Gyro} para o Studica NavX2 (porta MXP SPI por padrao).
 * O NavX mede yaw horario-positivo; aqui os sinais sao invertidos para a convencao CCW do WPILib.
 */
public class NavXGyro implements Gyro {
  private final AHRS navx;

  /** Offset de software (rad) aplicado ao yaw continuo. */
  private double yawOffsetRadians = 0.0;

  public NavXGyro(AHRS.NavXComType comType) {
    this.navx = new AHRS(comType);
  }

  /** Cria um NavX na porta MXP (SPI). */
  public NavXGyro() {
    this(AHRS.NavXComType.kMXP_SPI);
  }

  private double rawYawRadians() {
    return -Math.toRadians(navx.getAngle());
  }

  @Override
  public Rotation2d getYaw() {
    return new Rotation2d(rawYawRadians() + yawOffsetRadians);
  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(navx.getPitch());
  }

  @Override
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(navx.getRoll());
  }

  @Override
  public double getYawVelocityRadPerSec() {
    return -Math.toRadians(navx.getRate());
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    yawOffsetRadians = yaw.getRadians() - rawYawRadians();
  }

  @Override
  public boolean isConnected() {
    return navx.isConnected();
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.connected = navx.isConnected();
    inputs.yawRadians = rawYawRadians() + yawOffsetRadians;
    inputs.pitchRadians = Math.toRadians(navx.getPitch());
    inputs.rollRadians = Math.toRadians(navx.getRoll());
    inputs.yawVelocityRadPerSec = getYawVelocityRadPerSec();
  }

  @Override
  public Object getNativeSensor() {
    return navx;
  }
}
