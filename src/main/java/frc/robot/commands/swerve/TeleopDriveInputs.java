package frc.robot.commands.swerve;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;

/**
 * Converte os analogicos do piloto em velocidades. E compartilhado pelos dois modos de direcao
 * (FIELD e FREEDOM), que so diferem em COMO interpretam o resultado.
 *
 * <p>O resultado esta no referencial do "piloto": +X = frente, +Y = esquerda, +omega = anti-horario.
 *
 * <ul>
 *   <li>Analogico esquerdo: direcao e velocidade (zona morta no vetor e curva quadratica).
 *   <li>Analogico direito: giro.
 *   <li>RT (analogico): acelerador. Sem apertar anda a {@code kBaseSpeedFraction} da velocidade
 *       maxima; apertando ate o fim chega a 100%.
 * </ul>
 */
public class TeleopDriveInputs {
  private final DoubleSupplier forward;
  private final DoubleSupplier left;
  private final DoubleSupplier rotation;
  private final DoubleSupplier boost;

  /**
   * @param forward Frente/tras (-1..1, +1 = frente).
   * @param left Esquerda/direita (-1..1, +1 = esquerda).
   * @param rotation Giro (-1..1, +1 = anti-horario).
   * @param boost Acelerador (0..1), normalmente o RT do piloto.
   */
  public TeleopDriveInputs(
      DoubleSupplier forward,
      DoubleSupplier left,
      DoubleSupplier rotation,
      DoubleSupplier boost) {
    this.forward = forward;
    this.left = left;
    this.rotation = rotation;
    this.boost = boost;
  }

  /** @return Velocidades (m/s, m/s, rad/s) no referencial do piloto, ja com zona morta e curva. */
  public ChassisSpeeds get() {
    // Translacao: zona morta no VETOR (nao por eixo) e curva quadratica para precisao
    double x = forward.getAsDouble();
    double y = left.getAsDouble();
    double magnitude = MathUtil.applyDeadband(Math.min(Math.hypot(x, y), 1.0), kStickDeadband);
    magnitude = magnitude * magnitude;
    double angle = Math.atan2(y, x);

    // RT: de kBaseSpeedFraction (solto) ate 100% (apertado)
    double trigger = MathUtil.applyDeadband(MathUtil.clamp(boost.getAsDouble(), 0.0, 1.0), kBoostDeadband);
    double speedFraction = kBaseSpeedFraction + (1.0 - kBaseSpeedFraction) * trigger;

    double speed = magnitude * speedFraction * kMaxSpeedMps;

    // Giro: zona morta + curva quadratica
    double turn = MathUtil.applyDeadband(rotation.getAsDouble(), kStickDeadband);
    turn = Math.copySign(turn * turn, turn);

    return new ChassisSpeeds(
        speed * Math.cos(angle),
        speed * Math.sin(angle),
        turn * kRotationSpeedFraction * kMaxAngularSpeedRadPerSec);
  }

  /** @return Posicao do acelerador (RT) com zona morta, 0..1 (para log e dashboard). */
  public double getBoost() {
    return MathUtil.applyDeadband(MathUtil.clamp(boost.getAsDouble(), 0.0, 1.0), kBoostDeadband);
  }
}
